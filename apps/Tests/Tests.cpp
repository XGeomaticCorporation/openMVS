/*
 * Tests.cpp
 *
 * Copyright (c) 2014-2021 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include "../../libs/MVS/PointCloudSimplify.h"
#include "../../libs/Math/ConfidenceInterval.h"
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <tuple>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("Tests")


// S T R U C T S ///////////////////////////////////////////////////

static bool ComparePLY(const String& first, const String& second, size_t recordSize=0, bool subset=false)
{
	std::ifstream a(first.c_str(), std::ios::binary);
	std::ifstream b(second.c_str(), std::ios::binary);
	if (!a || !b)
		return false;
	std::string lineA, lineB;
	while (std::getline(a, lineA) && std::getline(b, lineB)) {
		if (lineA.compare(0, 15, "element vertex ") == 0 &&
			lineB.compare(0, 15, "element vertex ") == 0)
		{
			const auto countA = std::stoull(lineA.substr(15)), countB = std::stoull(lineB.substr(15));
			if (subset ? countB == 0 || countB > countA : countA != countB)
				return false;
		} else if (lineA != lineB) {
			return false;
		}
		if (lineA == "end_header") {
			const std::vector<char> dataA((std::istreambuf_iterator<char>(a)), std::istreambuf_iterator<char>());
			const std::vector<char> dataB((std::istreambuf_iterator<char>(b)), std::istreambuf_iterator<char>());
			if (recordSize == 0)
				return dataA == dataB;
			if ((!subset && dataA.size() != dataB.size()) ||
				dataA.size() % recordSize != 0 || dataB.size() % recordSize != 0)
				return false;
			const auto records = [recordSize](const std::vector<char>& data) {
				std::vector<std::string> result;
				result.reserve(data.size() / recordSize);
				for (size_t offset=0; offset<data.size(); offset+=recordSize)
					result.emplace_back(data.data() + offset, recordSize);
				std::sort(result.begin(), result.end());
				return result;
			};
			const auto sortedA = records(dataA), sortedB = records(dataB);
			return subset ? std::includes(sortedA.begin(),sortedA.end(),sortedB.begin(),sortedB.end()) : sortedA == sortedB;
		}
	}
	return false;
}

static bool TestStreamingPLY()
{
	std::error_code ec;
	const std::filesystem::path root(std::filesystem::temp_directory_path(ec));
	if (ec)
		return false;
	const String directoryName("openmvs-stream-test-" + Util::getUniqueName(0));
	const std::filesystem::path directory(root / directoryName.c_str());
	if (!std::filesystem::create_directory(directory, ec))
		return false;
	const String reference((directory / "reference.ply").string());
	const String streamed((directory / "streamed.ply").string());
	const String existing((directory / "existing.ply").string());
	bool ok = true;
	for (unsigned flags=0; flags<4 && ok; ++flags) {
		const bool colors = (flags & 1) != 0;
		const bool normals = (flags & 2) != 0;
		PointCloud points;
		for (unsigned i=0; i<4; ++i) {
			points.points.emplace_back(float(i)+0.25f, float(i)-0.5f, -float(i)-1.f);
			PointCloud::ViewArr& views = points.pointViews.AddEmpty();
			const unsigned numViews = i == 3 ? 260 : i+1;
			for (unsigned v=0; v<numViews; ++v)
				views.push_back(v);
			if (colors)
				points.colors.emplace_back(uint8_t(i+1), uint8_t(i+11), uint8_t(i+21));
			if (normals)
				points.normals.emplace_back(0.f, 0.f, -1.f);
		}
		if (!points.Save(reference, true, true)) {
			ok = false;
			break;
		}
		PointCloudStreamWriter writer(streamed, colors, normals);
		if (!writer.IsOpen()) {
			ok = false;
			break;
		}
		for (unsigned i=0; i<4 && ok; ++i) {
			const PointCloud::Color color(colors ? points.colors[i] : PointCloud::Color(0, 0, 0));
			const PointCloud::Normal normal(normals ? points.normals[i] : PointCloud::Normal(0, 0, 0));
			ok = writer.Write(points.points[i], color, normal, points.pointViews[i].size());
		}
		ok = ok && writer.GetSize() == 4 && writer.Finish() && ComparePLY(reference, streamed);
	}
	if (ok) {
		{
			std::ofstream out(existing.c_str(), std::ios::binary);
			out << "keep";
			ok = out.good();
		}
		if (ok) {
			PointCloudStreamWriter writer(existing, true, true);
			ok = writer.IsOpen() && writer.Write(PointCloud::Point(1, 2, 3), PointCloud::Color(1, 2, 3), PointCloud::Normal(0, 0, -1), 2);
		}
		if (ok) {
			PointCloudStreamWriter writer(existing, true, true);
			ok = writer.IsOpen() && !writer.Finish();
		}
		std::ifstream in(existing.c_str(), std::ios::binary);
		const std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		ok = ok && contents == "keep";
	}
	std::filesystem::remove(reference.c_str(), ec);
	std::filesystem::remove(streamed.c_str(), ec);
	std::filesystem::remove(existing.c_str(), ec);
	if (!std::filesystem::remove(directory, ec) || ec)
		ok = false;
	return ok;
}

static bool TestAdaptiveSimplification()
{
	Camera camera;
	camera.K = KMatrix::eye();
	camera.K(0,0) = 1000;
	camera.K(1,1) = 500;
	camera.R = RMatrix::eye();
	camera.C = Point3::ZERO;
	if (std::abs(SurfacePixelSize(camera,Point3d(0,0,100),Point3d(0,0,1))-0.1) > 1e-10 ||
		SurfacePixelSize(camera,Point3d(0,0,-1),Point3d(0,0,1)) != 0 ||
		SurfacePixelSize(camera,Point3d(0,0,100),Point3d(1,0,0)) != 0)
		return false;
	// Synthetic scene with GSD 0.1 and raw spacing 0.5 GSD: a flat area, a 90 degree ridge,
	// an isolated thin line, two parallel layers 3 GSD apart, a checker texture and points without GSD.
	enum Kind { FLAT, RIDGE, LINE, LAYER, TEXTURE, UNKNOWN };
	struct Sample { PointCloud::Point p; Kind kind; PointCloud::Color c = PointCloud::Color(100,100,100); };
	std::vector<Sample> samples;
	uint32_t seed = 12345;
	const auto jitter = [&seed]() {
		seed = seed*1664525u + 1013904223u;
		return (float(seed >> 8)/float(1u << 24) - 0.5f)*0.02f;
	};
	for (int y=0; y<160; ++y)
		for (int x=0; x<160; ++x)
			samples.push_back({PointCloud::Point(0.05f*x+jitter(),0.05f*y+jitter(),0.f),FLAT});
	for (int y=0; y<80; ++y)
		for (int x=0; x<80; ++x) {
			const float px = 10.f+0.05f*x+jitter();
			samples.push_back({PointCloud::Point(px,0.05f*y+jitter(),std::abs(px-12.f)),RIDGE});
		}
	for (int y=0; y<80; ++y)
		samples.push_back({PointCloud::Point(17.f,0.05f*y,0.3f),LINE});
	for (int layer=0; layer<2; ++layer)
		for (int y=0; y<80; ++y)
			for (int x=0; x<80; ++x)
				samples.push_back({PointCloud::Point(20.f+0.05f*x+jitter(),0.05f*y+jitter(),0.3f*layer),LAYER});
	for (int y=0; y<80; ++y)
		for (int x=0; x<80; ++x) {
			const uint8_t c = (x/4+y/4)%2 ? 230 : 20;
			samples.push_back({PointCloud::Point(40.f+0.05f*x+jitter(),0.05f*y+jitter(),0.f),TEXTURE,PointCloud::Color(c,c,c)});
		}
	for (int i=0; i<10; ++i)
		samples.push_back({PointCloud::Point(30.f+i,0.f,0.f),UNKNOWN});
	std::error_code ec;
	const auto root = std::filesystem::temp_directory_path(ec);
	if (ec)
		return false;
	const auto directory = root / ("openmvs-adaptive-test-" + Util::getUniqueName(0)).c_str();
	if (!std::filesystem::create_directory(directory,ec))
		return false;
	const String a((directory/"a.ply").string()), b((directory/"b.ply").string());
	const auto run = [&](const String& output, size_t budget, PointCloud& result) {
		{
			PointCloudStreamWriter writer(output,true,true);
			AdaptiveSimplifyOptions options;
			options.memoryBytes = budget;
			PointCloudAdaptiveSimplifier simplify(writer,options);
			if (!simplify.IsOpen())
				return false;
			for (const Sample& sample : samples)
				if (!simplify.Write(sample.p,sample.c,PointCloud::Normal(0,0,1),3,
					sample.kind == UNKNOWN ? 0. : 0.1))
					return false;
			if (simplify.GetInputSize() != samples.size() || !simplify.Finish() || !writer.Finish())
				return false;
		}
		result.Release();
		return result.Load(output);
	};
	const auto check = [&](const PointCloud& result, const char* name) {
		std::set<std::tuple<float,float,float>> original, retained;
		for (const Sample& sample : samples)
			original.emplace(sample.p.x,sample.p.y,sample.p.z);
		std::map<std::pair<int,int>,std::vector<PointCloud::Point>> index;
		for (const auto& p : result.points) {
			if (original.count(std::make_tuple(p.x,p.y,p.z)) != 1) {
				VERBOSE("ERROR: adaptive %s output contains a point not present in the input",name);
				return false;
			}
			retained.emplace(p.x,p.y,p.z);
			index[{int(std::floor(p.x/0.25f)),int(std::floor(p.y/0.25f))}].push_back(p);
		}
		const auto nearest = [&index](const PointCloud::Point& p, bool sameZ) {
			float best = std::numeric_limits<float>::max();
			for (int dy=-1; dy<=1; ++dy)
				for (int dx=-1; dx<=1; ++dx) {
					const auto it = index.find({int(std::floor(p.x/0.25f))+dx,int(std::floor(p.y/0.25f))+dy});
					if (it != index.end())
						for (const auto& q : it->second)
							if (!sameZ || std::abs(q.z-p.z) < 0.01f)
								best = std::min(best,float(norm(p-q)));
				}
			return best;
		};
		size_t flatIn = 0, flatOut = 0, ridgeIn = 0, ridgeOut = 0, slopeIn = 0, slopeOut = 0, textureIn = 0, textureOut = 0;
		for (const Sample& sample : samples) {
			const PointCloud::Point& p = sample.p;
			const bool kept = retained.count(std::make_tuple(p.x,p.y,p.z)) != 0;
			// Coverage bound: flat spacing (2 GSD) everywhere, 1 GSD on the line and 1.5 GSD on the texture.
			const float limit = (sample.kind == LINE ? 0.1f : sample.kind == TEXTURE ? 0.15f : 0.2f)*1.01f;
			if (sample.kind == UNKNOWN ? !kept : nearest(p,sample.kind == LAYER) > limit) {
				VERBOSE("ERROR: adaptive %s point (%g,%g,%g) kind %d is not covered",name,p.x,p.y,p.z,int(sample.kind));
				return false;
			}
			if (sample.kind == FLAT && p.x > 0.5f && p.x < 7.5f && p.y > 0.5f && p.y < 7.5f) {
				++flatIn;
				flatOut += kept;
			}
			if (sample.kind == RIDGE && p.y > 0.5f && p.y < 3.5f) {
				const float d = std::abs(p.x-12.f);
				if (d < 0.1f) { ++ridgeIn; ridgeOut += kept; }
				else if (d > 0.6f && d < 1.5f) { ++slopeIn; slopeOut += kept; }
			}
			if (sample.kind == TEXTURE && p.x > 40.5f && p.x < 43.5f && p.y > 0.5f && p.y < 3.5f) {
				++textureIn;
				textureOut += kept;
			}
		}
		const double flat = double(flatOut)/flatIn, ridge = double(ridgeOut)/ridgeIn,
			slope = double(slopeOut)/slopeIn, texture = double(textureOut)/textureIn;
		VERBOSE("Adaptive synthetic test (%s): %u -> %u points, kept flat %.1f%% slope %.1f%% ridge %.1f%% texture %.1f%%",
			name,unsigned(samples.size()),result.GetSize(),100*flat,100*slope,100*ridge,100*texture);
		return flat < 0.5 && slope < 0.5 && ridge > 1.2*slope && texture > 1.4*flat;
	};
	PointCloud large, small;
	bool ok = run(a,256*1024*1024,large) && check(large,"single tile") &&
		run(b,1024*1024,small) && check(small,"multi tile") &&
		std::abs(double(small.GetSize())-double(large.GetSize())) <= 0.05*double(large.GetSize());
	if (ok) {
		{
			std::ofstream out(a.c_str(),std::ios::binary);
			out << "keep";
		}
		{
			PointCloudStreamWriter writer(a,true,true);
			AdaptiveSimplifyOptions options;
			options.memoryBytes = 1;
			PointCloudAdaptiveSimplifier simplify(writer,options);
			ok = !simplify.IsOpen() && !simplify.Finish();
		}
		{
			PointCloudStreamWriter writer(a,true,true);
			PointCloudAdaptiveSimplifier simplify(writer,AdaptiveSimplifyOptions());
			ok = ok && simplify.IsOpen() && simplify.Write(samples.front().p,PointCloud::Color(1,2,3),PointCloud::Normal(0,0,1),2,0.1);
		}
		std::ifstream in(a.c_str(),std::ios::binary);
		ok = ok && std::string((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>()) == "keep";
	}
	std::filesystem::remove(a.c_str(),ec);
	std::filesystem::remove(b.c_str(),ec);
	if (!std::filesystem::remove(directory,ec) || ec)
		ok = false;
	return ok;
}

static bool TestStreamingFusion()
{
	std::error_code ec;
	const std::filesystem::path root(std::filesystem::temp_directory_path(ec));
	if (ec)
		return false;
	const String directoryName("openmvs-fusion-test-" + Util::getUniqueName(0));
	const std::filesystem::path directory(root / directoryName.c_str());
	if (!std::filesystem::create_directory(directory, ec))
		return false;
	const std::filesystem::path dataset(_DATA_PATH);
	std::filesystem::copy_file(dataset / "scene.mvs", directory / "scene.mvs", ec);
	if (!ec)
		std::filesystem::copy(dataset / "images", directory / "images", std::filesystem::copy_options::recursive, ec);
	if (ec)
		VERBOSE("ERROR: cannot prepare streaming fusion test data: %s", ec.message().c_str());

	const String previousFolder(WORKING_FOLDER);
	WORKING_FOLDER = directory.string();
	INIT_WORKING_FOLDER;
	bool ok = false;
	if (!ec) {
		ok = [&]() {
			#ifdef _USE_CUDA
			SEACAVE::CUDA::desiredDeviceID = -2;
			#endif
			OPTDENSE::init();
			OPTDENSE::nResolutionLevel = 3;
			OPTDENSE::nMinResolution = OPTDENSE::nMaxResolution = 256;
			OPTDENSE::nSubResolutionLevels = 0;
			OPTDENSE::nNumViews = 3;
			OPTDENSE::nMinViewsFuse = 2;
			OPTDENSE::nMinPixelsFuse = 3;
			OPTDENSE::nEstimationGeometricIters = 0;
			OPTDENSE::nOptimize = 0;
			OPTDENSE::nFuseFilter = OPTDENSE::FUSE_DENSEFILTER;
			OPTDENSE::bRemoveDmaps = false;
			{
				Scene scene(1);
				if (!scene.Load(MAKE_PATH("scene.mvs")) || !scene.DenseReconstruction(1, false))
					return false;
			}
			const String reference((directory / "reference.ply").string());
			const String streamed((directory / "streamed.ply").string());
			for (unsigned flags=0; flags<4; ++flags) {
				const bool colors = (flags & 1) != 0;
				const bool normals = (flags & 2) != 0;
				OPTDENSE::nEstimateColors = colors ? 2 : 0;
				OPTDENSE::nEstimateNormals = normals ? 2 : 0;
				for (unsigned crop=0; crop<2; ++crop) {
					{
						Scene scene(1);
						if (!scene.Load(MAKE_PATH("scene.mvs")) ||
							(crop && !scene.IsBounded() && !scene.EstimateROI(1.1f)) ||
							!scene.DenseReconstruction(0, crop != 0) || scene.pointcloud.IsEmpty() ||
							!scene.pointcloud.Save(reference, true, true))
							return false;
					}
					{
						Scene scene(1);
						PointCloudStreamWriter writer(streamed, colors, normals);
						if (!scene.Load(MAKE_PATH("scene.mvs")) ||
							(crop && !scene.IsBounded() && !scene.EstimateROI(1.1f)) ||
							!scene.DenseReconstruction(0, crop != 0, 0, 0, &writer, 64*1024*1024))
							return false;
					}
					const size_t recordSize = 3*sizeof(float) + (colors ? 3 : 0) + (normals ? 3*sizeof(float) : 0) + 1;
					if (!ComparePLY(reference, streamed, crop ? recordSize : 0))
						return false;
					{
						Scene scene(1);
						PointCloudStreamWriter writer(streamed,colors,normals);
						AdaptiveSimplifyOptions options;
						options.memoryBytes = 1024*1024;
						if (!scene.Load(MAKE_PATH("scene.mvs")) ||
							(crop && !scene.IsBounded() && !scene.EstimateROI(1.1f)) ||
							!scene.DenseReconstruction(0,crop != 0,0,0,&writer,64*1024*1024,&options) ||
							!ComparePLY(reference,streamed,recordSize,true))
							return false;
					}
				}
			}
			{
				// streaming fusion without depth-maps on disk estimates them first (last reference: colors, normals, ROI)
				for (const auto& entry : std::filesystem::directory_iterator(directory))
					if (entry.path().extension() == ".dmap" && !std::filesystem::remove(entry.path()))
						return false;
				Scene scene(1);
				PointCloudStreamWriter writer(streamed, true, true);
				if (!scene.Load(MAKE_PATH("scene.mvs")) ||
					(!scene.IsBounded() && !scene.EstimateROI(1.1f)) ||
					!scene.DenseReconstruction(0, true, 0, 0, &writer, 64*1024*1024))
					return false;
			}
			if (std::count_if(std::filesystem::directory_iterator(directory), std::filesystem::directory_iterator(),
					[](const std::filesystem::directory_entry& e) { return e.path().extension() == ".dmap"; }) == 0)
				return false;
			{
				// depth-map estimation is not deterministic, so only the point counts are compared
				const auto count = [](const String& file) {
					std::ifstream in(file.c_str(), std::ios::binary);
					for (std::string line; std::getline(in, line) && line != "end_header"; )
						if (line.compare(0, 15, "element vertex ") == 0)
							return double(std::stoull(line.substr(15)));
					return 0.;
				};
				const double a = count(reference), b = count(streamed);
				if (b == 0 || std::abs(a-b) > 0.1*a)
					return false;
			}
			const String existing((directory / "existing.ply").string());
			{
				std::ofstream out(existing.c_str(), std::ios::binary);
				out << "keep";
				if (!out)
					return false;
			}
			OPTDENSE::nEstimateColors = OPTDENSE::nEstimateNormals = 2;
			{
				Scene scene(1);
				PointCloudStreamWriter writer(existing, true, true);
				if (!scene.Load(MAKE_PATH("scene.mvs")) ||
					scene.DenseReconstruction(0, false, 0, 0, &writer, 1024*1024))
					return false;
			}
			{
				Scene scene(1);
				PointCloudStreamWriter writer(existing, false, true);
				if (!scene.Load(MAKE_PATH("scene.mvs")) ||
					scene.DenseReconstruction(0, false, 0, 0, &writer, 64*1024*1024))
					return false;
			}
			std::ifstream in(existing.c_str(), std::ios::binary);
			return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()) == "keep";
		}();
	}
	WORKING_FOLDER = previousFolder;
	INIT_WORKING_FOLDER;
	ec.clear();
	std::filesystem::remove_all(directory, ec);
	if (ec) {
		VERBOSE("ERROR: cannot clean streaming fusion test directory '%s': %s", directory.string().c_str(), ec.message().c_str());
		return false;
	}
	if (!ok)
		VERBOSE("ERROR: streaming fusion integration test failed!");
	return ok;
}

// test various algorithms independently
bool UnitTests()
{
	TD_TIMER_START();
	if (!TestStreamingPLY()) {
		VERBOSE("ERROR: streaming PLY test failed!");
		return false;
	}
	if (!TestAdaptiveSimplification()) {
		VERBOSE("ERROR: adaptive simplification test failed!");
		return false;
	}
	if (!SEACAVE::cListTest<true>(100)) {
		VERBOSE("ERROR: cListTest failed!");
		return false;
	}
	if (!SEACAVE::OctreeTest<double,2>(100)) {
		VERBOSE("ERROR: OctreeTest<double,2> failed!");
		return false;
	}
	if (!SEACAVE::OctreeTest<float,3>(100)) {
		VERBOSE("ERROR: OctreeTest<float,3> failed!");
		return false;
	}
	if (!SEACAVE::TestRayTriangleIntersection<float>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<float> failed!");
		return false;
	}
	if (!SEACAVE::TestRayTriangleIntersection<double>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<double> failed!");
		return false;
	}
	if (!TestConfidenceInterval()) {
		VERBOSE("ERROR: TestConfidenceInterval failed!");
		return false;
	}
	VERBOSE("All unit tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}


// test MVS stages on a small sample dataset
bool PipelineTest(bool verbose=false)
{
	TD_TIMER_START();
	#if 0 && defined(_USE_CUDA)
	// force CPU for testing even if CUDA is available
	SEACAVE::CUDA::desiredDeviceID = -2;
	#endif
	Scene scene;
	if (!scene.Load(MAKE_PATH("scene.mvs"))) {
		VERBOSE("ERROR: TestDataset failed loading the scene!");
		return false;
	}
	OPTDENSE::init();
	OPTDENSE::bRemoveDmaps = true;
	if (!scene.DenseReconstruction() || scene.pointcloud.GetSize() < 50000u) {
		VERBOSE("ERROR: TestDataset failed estimating dense point-cloud!");
		return false;
	}
	if (verbose)
		scene.pointcloud.Save(MAKE_PATH("scene_dense.ply"));
	if (!scene.ReconstructMesh() || scene.mesh.faces.size() < 25000u) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
	constexpr float decimate = 0.7f;
	scene.mesh.Clean(decimate);
	if (!ISINSIDE(scene.mesh.faces.size(), 18000u, 30000u)) {
		VERBOSE("ERROR: TestDataset failed cleaning the mesh!");
		return false;
	}
	#ifdef _USE_OPENMP
	TestMeshProjectionMT(scene.mesh, scene.images[1]);
	#endif
	if (!scene.TextureMesh(0, 0) || !scene.mesh.HasTexture()) {
		VERBOSE("ERROR: TestDataset failed texturing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture.ply"));
	VERBOSE("All pipeline stages passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// test OpenMVS functionality
int main(int argc, LPCTSTR* argv)
{
	OPEN_LOG();
	OPEN_LOGCONSOLE();
	if (argc > 2)
		OPEN_LOGFILE(argv[2]);
	MVS::Initialize(APPNAME);
	WORKING_FOLDER = _DATA_PATH;
	INIT_WORKING_FOLDER;
	if (argc > 1 && String(argv[1]) == "--stream-fusion") {
		if (!TestStreamingFusion())
			return EXIT_FAILURE;
	} else if (argc < 2 || std::atoi(argv[1]) == 0) {
		if (!UnitTests())
			return EXIT_FAILURE;
	} else {
		if (!PipelineTest())
			return EXIT_FAILURE;
	}
	MVS::Finalize();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
