#ifndef _MVS_POINTCLOUD_SIMPLIFY_H_
#define _MVS_POINTCLOUD_SIMPLIFY_H_

#include "PointCloud.h"
#include "Camera.h"

namespace MVS {

// Feature-adaptive point-cloud simplification; all lengths are in units of the local surface GSD.
struct MVS_API AdaptiveSimplifyOptions {
	double minSpacing = 1.0; // sample spacing on edges, corners, thin structures and texture
	double maxSpacing = 2.0; // sample spacing on flat, untextured surfaces
	double error = 0.5;      // maximum offset of a removed point from the retained point's tangent plane
	double transition = 30;  // distance over which the spacing may at most double (0 - no limit)
	size_t memoryBytes = 256 * 1024 * 1024;
	bool IsValid() const;
};

// World-space length per pixel in the best-resolved surface-tangent direction.
MVS_API double SurfacePixelSize(const Camera& camera, const Point3d& point, const Point3d& normal);

class MVS_API PointCloudAdaptiveSimplifier {
public:
	PointCloudAdaptiveSimplifier(PointCloudStreamWriter& writer, const AdaptiveSimplifyOptions& options);
	~PointCloudAdaptiveSimplifier();
	bool IsOpen() const;
	bool Write(const PointCloud::Point& point, const PointCloud::Color& color,
		const PointCloud::Normal& normal, unsigned views, double gsd);
	bool Finish();
	size_t GetInputSize() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl;
};

} // namespace MVS
#endif
