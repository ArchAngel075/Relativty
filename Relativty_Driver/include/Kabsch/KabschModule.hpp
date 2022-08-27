#pragma once
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "Kabsch/eigenutil.hpp"
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <thread>
#include <bitset>
#define MAX_MARKER_POINTS 64
/**
* Tracking
*/

//calibration ::

struct Marker2D
{
	int id;
	std::vector<Eigen::Vector2f> points;
	Marker2D(int id, int N) : id(id), points(N) {}
	Marker2D(int id, std::vector<Eigen::Vector2f>&& PTS)
	{ // Copy from temporary initializer
		this->id = id;
		points.swap(PTS);
	}
};

struct TransformCandidate
{
	Eigen::Isometry3f transform;
	float weight;
	std::vector<std::pair<Eigen::Isometry3f, float>> datapoints;
};

struct TransformSample
{
	Eigen::Isometry3f transform;
	float weight;
	int sampleCount;
	float stdDevT, stdDevR;
};

struct CameraRelation
{
	int camA;
	int camB;
	TransformSample sample; // Result
	std::vector<TransformCandidate> candidates;
};

////

//control ::

enum ControlPhase
{
	PHASE_None = 0,
	PHASE_Idle,
	PHASE_Calibration_Intrinsic,
	PHASE_Calibration_Extrinsic,
	PHASE_Calibration_Room,
	PHASE_Calibration_Marker,
	PHASE_Tracking
};

template <typename T>
struct IterativeParam
{
	T start, factor, summand, current;
	void reset() { current = start; }
	T get() { return current; }
	T iterate() { return current = current * factor + summand; }
};

/**
 * Camera State of a MarkerDetector with currently estimated transform, blobs and estimated poses
 */
struct CameraState
{
	Camera camera; // Calibration data
	std::vector<Eigen::Vector2f> points2D; // Points2D input
	std::vector<Eigen::Vector2f> undistorted2D; // undistorted points2D
	std::vector<float> pointSizes; // Points2D input
	struct { // Single-Camera state (Intrinsic calibration)
		std::vector<Marker2D> calibrationSelection;
		IterativeParam<float> selectionThreshold;
		IterativeParam<int> maxMarkerCount;
		// Current frame state
		std::vector<Marker2D> markers;
		std::vector<int> freeBlobs;
		// Radial marker condition
		std::multimap<float, uint16_t> radialLookup;
		IterativeParam<float> radialDensityGranularity;
		IterativeParam<float> radialDensityTarget;
		// Spatial marker condition
		std::vector<std::vector<uint16_t>> gridBuckets;
		int gridSize;
		IterativeParam<int> gridCountTarget;
		// Threaded calibration round
		std::vector<double> calibrationMarkerErrors;
		std::thread* calibrationThread;
		bool calibrationRunning;
		bool calibrationFinished;
		bool calibrationStopped;
		int calibrationMarkerCount;
		int calibrationTimeout;
	} intrinsic;
	struct { // Single-Camera state (Extrinsic calibration)
		// Extrinsic calibration state
		std::vector<CameraRelation*> relations;
		std::vector<TransformCandidate> originCandidates;
		TransformSample origin; // Result
		// Current frame state
		std::vector<Marker2D> markers;
		std::vector<int> freeBlobs;
		// Poses of current frame
		std::vector<Eigen::Isometry3f> poses;
		std::vector<float> posesMSE;
		std::vector<int> posesMatch;
		// Poses of last frame
		std::vector<Eigen::Isometry3f> lastPoses;
		std::vector<float> poseDiffRot;
		std::vector<Eigen::Vector3f> poseDiffPos;
	} extrinsic;
	struct { // Multi-Camera state (Tracking)
		std::vector<Ray> rays3D;
	} tracking;
	struct { // Ground-Truth values used in testing
		Camera camera;
		std::bitset<MAX_MARKER_POINTS> markerPtsVisible;
	} testing;
};
	
struct ControlState
{
	enum ControlPhase lastPhase;
	enum ControlPhase phase;
	std::vector<CameraState> cameras;
	struct { // Single-Camera state (Calibration)
		std::vector<DefMarker> markerTemplates2D;
		DefMarker* markerTemplate2D;
		std::vector<CameraRelation> relations;
		float maxRelationCandidateDiffT;
		float maxRelationCandidateDiffR;
		float maxOriginCandidateDiffT;
		float maxOriginCandidateDiffR;
		float maxPoseMSE;
		int phaseFocus;
	} calibration;
	struct { // Multi-Camera state (Tracking)
		std::vector<MarkerTemplate3D> markerTemplates3D;
		int trackID; // ID of currently tracked marker
		float minIntersectError;
		float maxIntersectError;
		float maxTemporalStaticErrorT;
		float maxTemporalStaticErrorR;
		float maxTemporalDynamicErrorT;
		float maxTemporalDynamicErrorR;
		float sigmaError;
		std::vector<TriangulatedPoint> points3D;
		int nonconflictedCount;
		std::vector<std::vector<int>> conflicts;
		std::vector<Eigen::Isometry3f> poses3D;
		std::vector<std::pair<float, int>> posesMSE;
	} tracking;
	struct { // Marker Calibration state
		MarkerTemplate3D iterativeMarker;
		int iteration;
		std::vector<Eigen::Vector3f> markerPoints;
		std::vector<std::pair<float, int>> markerPointRating;
	} markerCalib;
	struct { // Testing and visualization state
		bool isTesting;
		Eigen::Isometry3f targetPose;
		Eigen::Isometry3f GT;
		std::vector<Eigen::Vector3f> triangulatedPoints3D;
		float blobPxStdDev;
		DefMarker* markerTemplate3D;
	} testing;
};

////

/* Structures  */

struct Ray
{
	Eigen::Vector3f pos;
	Eigen::Vector3f dir;
	/* Exposed for performance reason, makes triangulateRayIntersections much easier */
	int intersectionCount;
	int conflictID; // if intersectionCount > 1
};

struct TriangulatedPoint
{
	Eigen::Vector3f pos;
	float error; // If valid, how accurate
	float confidence; // How likely to be valid
};

struct PointRelation
{
	int pt1;
	int pt2;
	Eigen::Vector3f dir;
	float distance;
};
struct ErrorRangeComp {
	float error;
	ErrorRangeComp(float error) { this->error = error; }
	bool operator() (const PointRelation& rel, float value);
	bool operator() (float value, const PointRelation& rel);
};

struct MarkerTemplate3D
{
	std::string label;
	int id;
	std::vector<Eigen::Vector3f> points;
	std::vector<PointRelation> relationDist; // Shortest neighouring relations of all points, sorted by distance
	std::vector<std::vector<int>> pointRelation; // Index of relations for each point
	// TODO: Change to continuous vector, as most subvectors have similar lengths (NUM_CLOSEST_RELATIONS to 2*NUM_CLOSEST_RELATIONS)
};

struct MarkerCandidate3D
{
	std::vector<int> points;
	std::vector<int> pointMap;
	// To get triangulated points: points3D[points[i]]
	// To get actual points: marker3D[pointMap[points3D[i]]]
	Eigen::Isometry3f pose;
};


/* Functions */

/**
	* Sets up lookup tables for quick marker identification
	*/
void generateLookupTables(MarkerTemplate3D* marker3D);

/**
	* Infer the pose of a marker given it's image points and the known camera position to transform into world space
	*/
void castRays(const std::vector<Eigen::Vector2f>& point2D, const Camera& camera, std::vector<Ray>& rays3D);

/**
	* Calculate the intersection points between rays of separate groups
	* Returns how many of the points are not conflicted (those are at the beginning of the array)
	*/
int triangulateRayIntersections(std::vector<std::vector<Ray>*>& rayGroups, std::vector<TriangulatedPoint>& points3D, std::vector<std::vector<int>>& conflicts, float maxError, float minError = 0.0f);

/**
	* Calculate MSE of candidate marker in given point cloud
	*/
float calculateCandidateMSE(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const MarkerCandidate3D* candidate);

/**
	* Picks the best candidate by point count and internal MSE.
	* Returns the index, MSE and count of other candidates with the same maximum point count
	*/
std::tuple<int, float, int> getBestMarkerCandidate(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const std::vector<std::vector<int>>& conflicts, int nonconflictedCount, const std::vector<MarkerCandidate3D>& candidates);

/**
	* Detect a marker in the triangulated 3D Point cloud, returns all candidates
	*/
void detectMarker3D(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const std::vector<std::vector<int>>& conflicts, int nonconflictedCount, std::vector<MarkerCandidate3D>& candidates, float sigmaError = 3, bool quickAssign = true);

/**
	* Detect a marker in the triangulated 3D Point cloud, returns the best candidate
	*/
float detectMarker3D(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const std::vector<std::vector<int>>& conflicts, int nonconflictedCount, MarkerCandidate3D* bestCandidate, float sigmaError = 3, bool quickAssign = true);

/**
	* Detect a marker in the triangulated 3D Point cloud, returns all candidate poses with respective MSE and point count
	*/
void detectMarker3D(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const std::vector<std::vector<int>>& conflicts, int nonconflictedCount, std::vector<Eigen::Isometry3f>& poses3D, std::vector<std::pair<float, int>>& posesMSE, float sigmaError = 3, bool quickAssign = true);

/**
	* Detect a marker in the triangulated 3D Point cloud, returns the best candidate pose with respective MSE and point count (point-count 0 if none found)
	*/
std::tuple<Eigen::Isometry3f, float, int> detectMarker3D(const MarkerTemplate3D* marker3D, const std::vector<TriangulatedPoint>& points3D, const std::vector<std::vector<int>>& conflicts, int nonconflictedCount, float sigmaError = 3, bool quickAssign = true);

/**
	* Matches the current poses to the poses of the last frame using temporal information
	*/
void matchTrackedPoses(const std::vector<Eigen::Isometry3f>& currentPose, const std::vector<Eigen::Isometry3f>& lastPose, const std::vector<Eigen::Vector3f>& lastDir, const std::vector<float>& lastRot, std::vector<int>& matching, float maxTemporalStaticErrorT = 10.0f, float maxTemporalStaticErrorR = 10.0f, float maxTemporalDynamicErrorT = 0.5f, float maxTemporalDynamicErrorR = 0.5f);

/**
	* Accepts the previously calculated matching and updates temporal information
	*/
void matchAccept(std::vector<Eigen::Isometry3f>& currentPose, std::vector<Eigen::Isometry3f>& lastPose, std::vector<Eigen::Vector3f>& lastDir, std::vector<float>& lastRot, const std::vector<int>& matching);
