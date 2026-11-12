#include "Common.h"
#include "PointCloudSimplify.h"
#include <array>
#include <filesystem>
#include <fstream>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <type_traits>

using namespace MVS;

bool AdaptiveSimplifyOptions::IsValid() const
{
	return std::isfinite(minSpacing) && std::isfinite(maxSpacing) &&
		minSpacing >= 0.5 && minSpacing <= maxSpacing && maxSpacing <= 4 &&
		std::isfinite(error) && error > 0 && error <= 2 &&
		std::isfinite(transition) && transition >= 0 && transition <= 1000 &&
		memoryBytes >= 1024 * 1024;
}

namespace {

bool Finite(const Point3d& p) {
	return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

void Tangents(const Point3d& normal, Point3d& u, Point3d& v) {
	u = normalized(normal.cross(std::abs(normal.x) < 0.8 ? Point3d(1,0,0) : Point3d(0,1,0)));
	v = normal.cross(u);
}

uint64_t Mix(uint64_t x) {
	x ^= x >> 30; x *= UINT64_C(0xbf58476d1ce4e5b9);
	x ^= x >> 27; x *= UINT64_C(0x94d049bb133111eb);
	return x ^ (x >> 31);
}

double SmoothStep(double x, double a, double b) {
	const double t = std::min(1., std::max(0., (x-a)/(b-a)));
	return t*t*(3-2*t);
}

double Ramp(double x, double a, double b) {
	return std::min(1., std::max(0., (x-a)/(b-a)));
}

// neighborhood radius of the feature analysis, in local GSD
constexpr double featureRadius = 3;

struct Record {
	PointCloud::Point point;
	PointCloud::Normal normal;
	float gsd;
	uint32_t views;
	uint64_t id;
	PointCloud::Color color;
	uint8_t halo;
};
static_assert(std::is_trivially_copyable<Record>::value, "simplification scratch records must be trivially copyable");

// approximate working memory per loaded point (records, sort buffers, features, voxels, flags)
constexpr size_t bytesPerPoint = 224;
constexpr int gridBits = 21;
constexpr int64_t gridMax = (int64_t(1) << gridBits) - 1;

uint64_t Pack(int64_t x, int64_t y, int64_t z) {
	return (uint64_t(x) << (2*gridBits)) | (uint64_t(y) << gridBits) | uint64_t(z);
}

struct TileFile {
	std::filesystem::path path;
	size_t count = 0, core = 0;
};

} // namespace

double MVS::SurfacePixelSize(const Camera& camera, const Point3d& point, const Point3d& normal)
{
	if (!Finite(point) || !Finite(normal) || normSq(normal) < 0.99 || normSq(normal) > 1.01)
		return 0;
	const Point3d p(camera.TransformPointW2C(point));
	if (!Finite(p) || p.z <= 0)
		return 0;
	Point3d u, v;
	Tangents(normal, u, v);
	const Point3d a(camera.R * u), b(camera.R * v);
	const double ax = camera.K(0,0) * (a.x - p.x/p.z*a.z) / p.z;
	const double ay = camera.K(1,1) * (a.y - p.y/p.z*a.z) / p.z;
	const double bx = camera.K(0,0) * (b.x - p.x/p.z*b.z) / p.z;
	const double by = camera.K(1,1) * (b.y - p.y/p.z*b.z) / p.z;
	const double aa = ax*ax + ay*ay, bb = bx*bx + by*by, ab = ax*bx + ay*by;
	const double delta = std::hypot(aa-bb, 2*ab);
	const double hi = (aa+bb+delta)/2, lo = (aa+bb-delta)/2;
	if (!std::isfinite(hi) || hi <= 0 || lo <= hi*0.01)
		return 0;
	return 1/std::sqrt(hi);
}

struct PointCloudAdaptiveSimplifier::Impl {
	PointCloudStreamWriter& writer;
	AdaptiveSimplifyOptions options;
	std::filesystem::path directory, spoolPath;
	std::ofstream spool;
	std::set<std::filesystem::path> paths;
	std::map<int64_t, TileFile> tileFiles;
	// accepted points near the border of already processed tiles: id -> sampling radius
	std::map<int64_t, std::unordered_map<uint64_t,float>> borders;
	size_t input = 0, spooled = 0, bypass = 0, removed = 0, featurePoints = 0, tiles = 0, maxTilePoints = 0;
	double minGSD = std::numeric_limits<double>::max(), maxGSD = 0;
	double bounds[4] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
	std::array<size_t,4096> histogram{};
	double g0 = 0, gcap = 0, halo = 0, tileSize = 0;
	int64_t nx = 0, ny = 0;
	bool ready = false, finished = false, failed = false, ownsDirectory = false;

	Impl(PointCloudStreamWriter& w, const AdaptiveSimplifyOptions& o) : writer(w), options(o) {
		if (!writer.IsOpen() || !options.IsValid()) {
			Error("invalid adaptive simplification options or output writer");
			return;
		}
		directory = std::filesystem::path(writer.GetScratchPath().c_str()).parent_path() / "simplify";
		std::error_code ec;
		if (!std::filesystem::create_directory(directory, ec)) {
			Error("cannot create simplification scratch directory");
			return;
		}
		ownsDirectory = true;
		spoolPath = directory / "spool.bin";
		paths.insert(spoolPath);
		spool.open(spoolPath, std::ios::binary);
		ready = spool.good();
		if (!ready)
			Error("cannot open simplification scratch file");
	}

	~Impl() {
		spool.close();
		std::error_code ec;
		for (const auto& path : paths) {
			std::filesystem::remove(path, ec);
			if (ec)
				VERBOSE("warning: cannot remove simplification scratch file '%s': %s", path.string().c_str(), ec.message().c_str());
			ec.clear();
		}
		if (ownsDirectory) {
			std::filesystem::remove(directory, ec);
			if (ec)
				VERBOSE("warning: cannot remove simplification scratch directory '%s': %s", directory.string().c_str(), ec.message().c_str());
		}
	}

	bool Error(const char* message) {
		VERBOSE("error: %s (%s)", message, directory.string().c_str());
		failed = true;
		return false;
	}

	bool Erase(const std::filesystem::path& path) {
		std::error_code ec;
		if (!std::filesystem::remove(path, ec) || ec)
			return Error("cannot remove processed simplification scratch file");
		paths.erase(path);
		return true;
	}

	bool Emit(const Record& r) {
		if (!writer.Write(r.point, r.color, r.normal, r.views))
			return Error("cannot write simplified point-cloud");
		return true;
	}

	bool Write(const PointCloud::Point& point, const PointCloud::Color& color,
		const PointCloud::Normal& normal, unsigned views, double gsd)
	{
		if (!ready || finished || failed)
			return Error("adaptive simplifier is not writable");
		if (!Finite(Point3d(point)))
			return Error("non-finite point supplied to adaptive simplifier");
		Record r{};
		r.point = point; r.normal = normal; r.color = color;
		r.views = views; r.id = input++;
		r.gsd = float(gsd);
		if (!std::isfinite(gsd) || gsd <= 0 || !std::isfinite(r.gsd) || r.gsd <= 0 ||
			!Finite(Point3d(normal)) || normSq(normal) < 0.99 || normSq(normal) > 1.01)
		{
			// unknown scale: kept unchanged
			++bypass;
			return Emit(r);
		}
		minGSD = std::min(minGSD, gsd); maxGSD = std::max(maxGSD, gsd);
		++histogram[std::max(0, std::min(4095, int((std::log2(gsd)+160)*12.8)))];
		bounds[0] = std::min(bounds[0], double(point.x)); bounds[1] = std::min(bounds[1], double(point.y));
		bounds[2] = std::max(bounds[2], double(point.x)); bounds[3] = std::max(bounds[3], double(point.y));
		spool.write(reinterpret_cast<const char*>(&r), sizeof(r));
		if (!spool.good())
			return Error("cannot write simplification scratch file (check free disk space)");
		++spooled;
		return true;
	}

	double Quantile(double q) const {
		size_t count = 0;
		for (size_t n : histogram)
			count += n;
		if (!count)
			return 0;
		const size_t target = size_t(q*double(count-1));
		size_t accumulated = 0;
		for (size_t i=0; i<histogram.size(); ++i) {
			accumulated += histogram[i];
			if (accumulated > target)
				return std::exp2((double(i)+0.5)/12.8-160);
		}
		return 0;
	}

	void TileOf(const PointCloud::Point& p, int64_t& tx, int64_t& ty) const {
		tx = std::max(int64_t(0), std::min(nx-1, int64_t(std::floor((double(p.x)-bounds[0])/tileSize))));
		ty = std::max(int64_t(0), std::min(ny-1, int64_t(std::floor((double(p.y)-bounds[1])/tileSize))));
	}

	// horizontal distance from a point to tile (tx,ty); the outer tiles extend to infinity
	double DistanceToTile(const PointCloud::Point& p, int64_t tx, int64_t ty) const {
		const double x0 = tx == 0 ? -std::numeric_limits<double>::max() : bounds[0]+double(tx)*tileSize;
		const double x1 = tx == nx-1 ? std::numeric_limits<double>::max() : bounds[0]+double(tx+1)*tileSize;
		const double y0 = ty == 0 ? -std::numeric_limits<double>::max() : bounds[1]+double(ty)*tileSize;
		const double y1 = ty == ny-1 ? std::numeric_limits<double>::max() : bounds[1]+double(ty+1)*tileSize;
		const double dx = std::max(0., std::max(x0-double(p.x), double(p.x)-x1));
		const double dy = std::max(0., std::max(y0-double(p.y), double(p.y)-y1));
		return std::hypot(dx, dy);
	}

	// distance from a point inside tile (tx,ty) to its nearest inner tile edge
	double DistanceToTileEdge(const PointCloud::Point& p, int64_t tx, int64_t ty) const {
		double d = std::numeric_limits<double>::max();
		if (tx > 0) d = std::min(d, double(p.x)-(bounds[0]+double(tx)*tileSize));
		if (tx < nx-1) d = std::min(d, bounds[0]+double(tx+1)*tileSize-double(p.x));
		if (ty > 0) d = std::min(d, double(p.y)-(bounds[1]+double(ty)*tileSize));
		if (ty < ny-1) d = std::min(d, bounds[1]+double(ty+1)*tileSize-double(p.y));
		return d;
	}

	// split the spool into horizontal tiles, each with a halo copy of nearby points of the neighbor tiles
	bool Partition() {
		const int64_t numTiles = nx*ny;
		constexpr int64_t group = 256;
		std::vector<Record> buffer(65536);
		for (int64_t t0=0; t0<numTiles; t0+=group) {
			const int64_t t1 = std::min(numTiles, t0+group);
			std::vector<std::unique_ptr<std::ofstream>> outs(size_t(t1-t0));
			std::ifstream in(spoolPath, std::ios::binary);
			if (!in)
				return Error("cannot read simplification scratch file");
			for (size_t remaining=spooled; remaining; ) {
				const size_t n = std::min(remaining, buffer.size());
				if (!in.read(reinterpret_cast<char*>(buffer.data()), std::streamsize(n*sizeof(Record))))
					return Error("truncated simplification scratch file");
				remaining -= n;
				for (size_t i=0; i<n; ++i) {
					int64_t tx, ty;
					TileOf(buffer[i].point, tx, ty);
					for (int64_t ax=std::max(int64_t(0),tx-1); ax<=std::min(nx-1,tx+1); ++ax)
						for (int64_t ay=std::max(int64_t(0),ty-1); ay<=std::min(ny-1,ty+1); ++ay) {
							const int64_t t = ax*ny+ay;
							if (t < t0 || t >= t1)
								continue;
							Record r(buffer[i]);
							r.halo = ax != tx || ay != ty;
							if (r.halo && DistanceToTile(r.point, ax, ay) > halo)
								continue;
							auto& out = outs[size_t(t-t0)];
							TileFile& info = tileFiles[t];
							if (!out) {
								info.path = directory / ("tile-" + std::to_string(t) + ".bin");
								paths.insert(info.path);
								out.reset(new std::ofstream(info.path, std::ios::binary));
								if (!*out)
									return Error("cannot create simplification tile");
							}
							out->write(reinterpret_cast<const char*>(&r), sizeof(r));
							if (!*out)
								return Error("cannot write simplification tile (check free disk space)");
							++info.count;
							if (!r.halo)
								++info.core;
						}
				}
			}
			if (in.peek() != std::char_traits<char>::eof())
				return Error("invalid simplification scratch file length");
			for (auto& out : outs)
				if (out) {
					out->close();
					if (!*out)
						return Error("cannot flush simplification tile");
				}
		}
		return Erase(spoolPath);
	}

	// Limit the spatial gradient of log(spacing): the spacing may at most double over options.transition GSD.
	// Exact distance transform (Dijkstra) over the occupied voxels, so it follows the surface and never crosses gaps.
	void LimitSpacingGradient(const std::vector<Record>& records, const Point3d& origin, double extent, std::vector<float>& radius) const {
		const size_t n = records.size();
		double cell = g0;
		while (extent/cell >= double(gridMax-1))
			cell *= 2;
		const double slope = std::log(2.)/(options.transition*g0);
		const auto index = [&](double v, double o) {
			return std::max(int64_t(0), std::min(gridMax, int64_t(std::floor((v-o)/cell))));
		};
		std::vector<uint64_t> keys(n);
		for (size_t i=0; i<n; ++i) {
			const Point3d p(records[i].point);
			keys[i] = Pack(index(p.x,origin.x), index(p.y,origin.y), index(p.z,origin.z));
		}
		std::vector<uint64_t> voxels(keys);
		std::sort(voxels.begin(), voxels.end());
		voxels.erase(std::unique(voxels.begin(), voxels.end()), voxels.end());
		const auto find = [&voxels](uint64_t key) {
			const auto it = std::lower_bound(voxels.begin(), voxels.end(), key);
			return it != voxels.end() && *it == key ? size_t(it-voxels.begin()) : voxels.size();
		};
		std::vector<uint32_t> voxelOf(n);
		std::vector<double> value(voxels.size(), std::numeric_limits<double>::max());
		for (size_t i=0; i<n; ++i) {
			voxelOf[i] = uint32_t(find(keys[i]));
			value[voxelOf[i]] = std::min(value[voxelOf[i]], std::log(double(radius[i])));
		}
		keys = std::vector<uint64_t>();
		typedef std::pair<double,uint32_t> Item;
		std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
		for (size_t v=0; v<voxels.size(); ++v)
			heap.emplace(value[v], uint32_t(v));
		constexpr uint64_t mask = (uint64_t(1) << gridBits)-1;
		while (!heap.empty()) {
			const Item item = heap.top();
			heap.pop();
			if (item.first > value[item.second])
				continue;
			const uint64_t key = voxels[item.second];
			const int64_t x = int64_t(key >> (2*gridBits)), y = int64_t((key >> gridBits) & mask), z = int64_t(key & mask);
			for (int dx=-1; dx<=1; ++dx)
				for (int dy=-1; dy<=1; ++dy)
					for (int dz=-1; dz<=1; ++dz) {
						if (!dx && !dy && !dz)
							continue;
						const int64_t ax = x+dx, ay = y+dy, az = z+dz;
						if (ax < 0 || ay < 0 || az < 0 || ax > gridMax || ay > gridMax || az > gridMax)
							continue;
						const size_t v = find(Pack(ax,ay,az));
						if (v == voxels.size())
							continue;
						const double candidate = item.first + slope*cell*std::sqrt(double(dx*dx+dy*dy+dz*dz));
						if (candidate < value[v]) {
							value[v] = candidate;
							heap.emplace(candidate, uint32_t(v));
						}
					}
		}
		// half voxel diagonal bounds the variation inside a voxel
		const double inside = slope*cell*0.87;
		for (size_t i=0; i<n; ++i)
			radius[i] = std::min(radius[i], float(std::exp(value[voxelOf[i]]+inside)));
	}

	bool ProcessTile(int64_t t, const TileFile& info) {
		const int64_t tx = t/ny, ty = t%ny;
		// only the previous and current tile columns can still be neighbors
		while (!borders.empty() && borders.begin()->first/ny < tx-1)
			borders.erase(borders.begin());
		if (!info.core)
			return Erase(info.path);
		std::vector<Record> records(info.count);
		{
			std::ifstream in(info.path, std::ios::binary);
			in.read(reinterpret_cast<char*>(records.data()), std::streamsize(records.size()*sizeof(Record)));
			if (!in || in.peek() != std::char_traits<char>::eof())
				return Error("cannot read complete simplification tile");
		}
		if (!Erase(info.path))
			return false;
		++tiles;
		maxTilePoints = std::max(maxTilePoints, records.size());
		const size_t n = records.size();

		// spatial grid: records sorted by cell, cells in (x,y,z) order so z-ranges are contiguous
		double zmin = std::numeric_limits<double>::max(), zmax = -zmin;
		for (const Record& r : records) {
			zmin = std::min(zmin, double(r.point.z));
			zmax = std::max(zmax, double(r.point.z));
		}
		const Point3d origin(bounds[0]+double(tx)*tileSize-halo, bounds[1]+double(ty)*tileSize-halo, zmin);
		double extent = zmax-zmin;
		for (const Record& r : records)
			extent = std::max(extent, std::max(double(r.point.x)-origin.x, double(r.point.y)-origin.y));
		double cell = 2*g0;
		while (extent/cell >= double(gridMax-1))
			cell *= 2;
		const auto cellIndex = [&](double v, double o) {
			return std::max(int64_t(0), std::min(gridMax, int64_t(std::floor((v-o)/cell))));
		};
		const auto key = [&](const Point3d& p) {
			return Pack(cellIndex(p.x,origin.x), cellIndex(p.y,origin.y), cellIndex(p.z,origin.z));
		};
		std::vector<uint64_t> cellKeys;
		std::vector<size_t> cellStart;
		{
			std::vector<std::pair<uint64_t,uint32_t>> order(n);
			for (size_t i=0; i<n; ++i)
				order[i] = {key(Point3d(records[i].point)), uint32_t(i)};
			std::sort(order.begin(), order.end(), [&](const auto& a, const auto& b) {
				return a.first < b.first || (a.first == b.first && records[a.second].id < records[b.second].id);
			});
			std::vector<Record> sorted(n);
			for (size_t i=0; i<n; ++i) {
				sorted[i] = records[order[i].second];
				if (!i || order[i].first != order[i-1].first) {
					cellKeys.push_back(order[i].first);
					cellStart.push_back(i);
				}
			}
			cellStart.push_back(n);
			records.swap(sorted);
		}
		// visit all records within the given radius; stops when the visitor returns true
		const auto forEach = [&](const Point3d& p, double radius, auto&& visit) {
			const int64_t x0 = cellIndex(p.x-radius,origin.x), x1 = cellIndex(p.x+radius,origin.x);
			const int64_t y0 = cellIndex(p.y-radius,origin.y), y1 = cellIndex(p.y+radius,origin.y);
			const int64_t z0 = cellIndex(p.z-radius,origin.z), z1 = cellIndex(p.z+radius,origin.z);
			const double radiusSq = radius*radius;
			for (int64_t x=x0; x<=x1; ++x)
				for (int64_t y=y0; y<=y1; ++y) {
					const uint64_t hi = Pack(x,y,z1);
					for (auto it=std::lower_bound(cellKeys.begin(), cellKeys.end(), Pack(x,y,z0)); it!=cellKeys.end() && *it<=hi; ++it) {
						const size_t c = size_t(it-cellKeys.begin());
						for (size_t j=cellStart[c]; j<cellStart[c+1]; ++j) {
							const Point3d d(Point3d(records[j].point)-p);
							if (normSq(d) <= radiusSq && visit(j, d))
								return;
						}
					}
				}
		};
		const auto localGSD = [&](const Record& r) { return std::min(double(r.gsd), gcap); };

		// feature score: 0 - flat untextured surface, 1 - edge, corner, line or strong texture;
		// the neighborhood is wide enough (3 GSD) that depth noise alone stays near the flat end
		std::vector<float> score(n), radius(n), sigma(n);
		std::vector<Point3f> normals(n), centroids(n);
		const bool colors = writer.HasColors();
		const double minSpacing = options.minSpacing, maxSpacing = options.maxSpacing;
		#ifdef _USE_OPENMP
		#pragma omp parallel for schedule(dynamic, 1024)
		#endif
		for (int64_t i=0; i<int64_t(n); ++i) {
			const Record& r = records[size_t(i)];
			const Point3d p(r.point);
			const double g = localGSD(r);
			size_t count = 0;
			Point3d s1(0,0,0);
			double s2[6] = {0,0,0,0,0,0}, l1 = 0, l2 = 0;
			forEach(p, featureRadius*g, [&](size_t j, const Point3d& d) {
				++count;
				s1 += d;
				s2[0] += d.x*d.x; s2[1] += d.x*d.y; s2[2] += d.x*d.z;
				s2[3] += d.y*d.y; s2[4] += d.y*d.z; s2[5] += d.z*d.z;
				const PointCloud::Color& c = records[j].color;
				const double l = 0.299*c.r + 0.587*c.g + 0.114*c.b;
				l1 += l; l2 += l*l;
				return false;
			});
			double s = 1, deviation = 0;
			Point3f normal(r.normal);
			Point3d centroid(p);
			if (count >= 3) {
				const double k = 1./double(count);
				const Point3d m(s1*k);
				const cv::Matx33d covariance(
					s2[0]*k-m.x*m.x, s2[1]*k-m.x*m.y, s2[2]*k-m.x*m.z,
					s2[1]*k-m.x*m.y, s2[3]*k-m.y*m.y, s2[4]*k-m.y*m.z,
					s2[2]*k-m.x*m.z, s2[4]*k-m.y*m.z, s2[5]*k-m.z*m.z);
				cv::Vec3d ev;
				cv::Matx33d evec;
				if (cv::eigen(covariance, ev, evec)) {
					const double l0 = std::max(ev[2], 0.), lmid = std::max(ev[1], 0.), lmax = std::max(ev[0], 0.);
					const double total = l0+lmid+lmax;
					if (total > 0) {
						const double variation = l0/total;
						const double linearity = (lmax-lmid)/lmax;
						const double lumaStd = std::sqrt(std::max(0., l2*k-(l1*k)*(l1*k)));
						// linear ramps avoid a near-binary score and therefore density steps
						s = std::max(Ramp(variation, 0.03, 0.25), SmoothStep(linearity, 0.7, 0.9));
						if (colors)
							s = std::max(s, 0.5*Ramp(lumaStd, 10., 50.));
						normal = Point3f(float(evec(2,0)), float(evec(2,1)), float(evec(2,2)));
						centroid = p+m;
						deviation = std::sqrt(l0);
					}
				}
			}
			score[size_t(i)] = float(s);
			normals[size_t(i)] = normal;
			centroids[size_t(i)] = Point3f(centroid);
			sigma[size_t(i)] = float(deviation);
			radius[size_t(i)] = float(g*(minSpacing+(maxSpacing-minSpacing)*(1-s)));
		}
		if (options.transition > 0)
			LimitSpacingGradient(records, origin, extent, radius);

		// halo points already decided by previous tiles keep their decision and radius
		std::vector<uint8_t> accepted(n, 0);
		std::vector<uint32_t> queue;
		queue.reserve(info.core);
		for (size_t i=0; i<n; ++i) {
			const Record& r = records[i];
			if (!r.halo) {
				queue.push_back(uint32_t(i));
				continue;
			}
			int64_t hx, hy;
			TileOf(r.point, hx, hy);
			const auto border = borders.find(hx*ny+hy);
			if (border == borders.end())
				continue;
			const auto it = border->second.find(r.id);
			if (it != border->second.end()) {
				accepted[i] = 1;
				radius[i] = it->second;
			}
		}
		// variable-radius Poisson-disk selection of original points; features are visited first
		std::sort(queue.begin(), queue.end(), [&](uint32_t a, uint32_t b) {
			if (score[a] != score[b])
				return score[a] > score[b];
			if (records[a].views != records[b].views)
				return records[a].views > records[b].views;
			const uint64_t ha = Mix(records[a].id), hb = Mix(records[b].id);
			return ha < hb || (ha == hb && records[a].id < records[b].id);
		});
		for (uint32_t i : queue) {
			const Record& r = records[i];
			const double rp = radius[i];
			const double tolerance = options.error*localGSD(r);
			const double floorSq = minSpacing*localGSD(r)*minSpacing*localGSD(r);
			// thickness of the noisy surface band treated as a single layer
			const double band = std::min(tolerance+2.5*double(sigma[i]), rp);
			const Point3d normal(normals[i]), centroid(centroids[i]);
			bool covered = false;
			forEach(Point3d(r.point), std::sqrt(rp*rp+band*band), [&](size_t j, const Point3d& d) {
				if (!accepted[j])
					return false;
				const double rr = std::min(rp, double(radius[j]));
				const double distSq = normSq(d), offset = normal.dot(d);
				// spacing is measured along the surface so that a thick noisy layer is not kept as several layers
				if (distSq-offset*offset > rr*rr)
					return false;
				// a point off the local surface (another layer or a small detail) is kept,
				// unless it is closer than the minimum spacing
				covered = distSq <= floorSq || (std::abs(offset) <= band &&
					std::abs(normal.dot(Point3d(centroids[j])-centroid)) <= tolerance);
				return covered;
			});
			if (!covered)
				accepted[i] = 1;
		}
		auto& border = borders[t];
		for (size_t i=0; i<n; ++i) {
			const Record& r = records[i];
			if (r.halo)
				continue;
			if (score[i] > 0.5f)
				++featurePoints;
			if (!accepted[i]) {
				++removed;
				continue;
			}
			if (!Emit(r))
				return false;
			if (DistanceToTileEdge(r.point, tx, ty) <= halo)
				border.emplace(r.id, radius[i]);
		}
		return true;
	}

	bool Simplify() {
		g0 = Quantile(0.5);
		if (!(g0 > 0))
			return Error("invalid dense surface GSD");
		gcap = 2*g0;
		// a core point depends on: the spacing transition (points up to log2(max/min)*transition away),
		// the features of those points (one feature radius) and the Poisson-disk neighbors
		const double transition = options.transition > 0 ? std::log2(options.maxSpacing/options.minSpacing)*options.transition*g0 + 2*g0 : 0;
		halo = (transition + featureRadius*gcap + std::sqrt(2.)*options.maxSpacing*gcap)*1.01;
		const double width = std::max(bounds[2]-bounds[0], halo), height = std::max(bounds[3]-bounds[1], halo);
		const double target = std::max(double(options.memoryBytes/bytesPerPoint), 4096.);
		tileSize = std::max(std::sqrt(width*height*target/double(spooled)), 4*halo);
		nx = int64_t(width/tileSize)+1;
		ny = int64_t(height/tileSize)+1;
		if (nx > (int64_t(1)<<20) || ny > (int64_t(1)<<20))
			return Error("too many simplification tiles");
		if (!Partition())
			return false;
		while (!tileFiles.empty()) {
			const auto it = tileFiles.begin();
			if (!ProcessTile(it->first, it->second))
				return false;
			tileFiles.erase(it);
		}
		return true;
	}

	bool Finish() {
		if (!ready || finished || failed)
			return Error("adaptive simplifier cannot be finalized");
		TD_TIMER_STARTD();
		spool.close();
		if (!spool)
			return Error("cannot flush simplification scratch file");
		if (spooled ? !Simplify() : !Erase(spoolPath))
			return false;
		finished = true;
		VERBOSE("Adaptive simplification: %llu -> %llu points (%.2f%% removed), %.2f%% feature points, %llu unknown-scale points kept (%s)",
			static_cast<unsigned long long>(input), static_cast<unsigned long long>(input-removed),
			input ? 100.*double(removed)/double(input) : 0., spooled ? 100.*double(featurePoints)/double(spooled) : 0.,
			static_cast<unsigned long long>(bypass), TD_TIMER_GET_FMT().c_str());
		if (spooled)
			VERBOSE("Adaptive simplification tiles: %llu tiles of %.6g x %.6g, halo %.6g, largest %llu points (including halo)",
				static_cast<unsigned long long>(tiles), tileSize, tileSize, halo, static_cast<unsigned long long>(maxTilePoints));
		if (maxGSD > 0)
			VERBOSE("Dense surface GSD (scene units): min %.6g, approximate p10/p50/p90 %.6g/%.6g/%.6g, max %.6g",
				minGSD, Quantile(0.1), Quantile(0.5), Quantile(0.9), maxGSD);
		return true;
	}
};

PointCloudAdaptiveSimplifier::PointCloudAdaptiveSimplifier(PointCloudStreamWriter& writer, const AdaptiveSimplifyOptions& options)
	: impl(new Impl(writer,options)) {}
PointCloudAdaptiveSimplifier::~PointCloudAdaptiveSimplifier() = default;
bool PointCloudAdaptiveSimplifier::IsOpen() const { return impl->ready && !impl->failed && !impl->finished; }
bool PointCloudAdaptiveSimplifier::Write(const PointCloud::Point& p, const PointCloud::Color& c,
	const PointCloud::Normal& n, unsigned views, double gsd) { return impl->Write(p,c,n,views,gsd); }
bool PointCloudAdaptiveSimplifier::Finish() { return impl->Finish(); }
size_t PointCloudAdaptiveSimplifier::GetInputSize() const { return impl->input; }
