#include <iostream>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING 1 // screw you boost I never asked for this
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <infotaxis.hpp>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static auto static_map = 		nav_msgs::OccupancyGrid::ConstPtr();
static auto static_grid = 		boost::shared_ptr<infotaxis::InfotaxisGrid>();
static auto static_local_costmap  =	nav_msgs::OccupancyGrid::ConstPtr();
static auto static_global_costmap = nav_msgs::OccupancyGrid::ConstPtr();
static tf::TransformListener 	*static_tfl;
//static Publisher grid_publisher;

enum RobotStates {
	STATE_EXIT = 0,
	STATE_DETECTING = 1,
	STATE_PLANNING = 2,
	STATE_MOVING = 3
};

static const char* STATE_NAMES[] = {"EXIT", "DETECTING", "PLANNING", "MOVING"};

enum SubscribedMapTypes {
	STATIC_MAP,
	LOCAL_COSTMAP,
	GLOBAL_COSTMAP
};

void cmd_callback(const nav_msgs::OccupancyGrid::ConstPtr& map, SubscribedMapTypes type) {
	ROS_DEBUG("Received %smap", type == STATIC_MAP ? "" : "cost");
	if (map) {
		ROS_DEBUG("seq:%u, res:%f, size:%ux%u, pos:%f:%f:%f\n", map->header.seq, map->info.resolution, map->info.width, map->info.height, map->info.origin.position.x, map->info.origin.position.y, map->info.origin.position.z);
		if (type == STATIC_MAP) {
			if (static_map && (static_map->info.width != map->info.width || static_map->info.height != map->info.height)) {
				ROS_FATAL("Map updated with different dimensions : unsupported. Shutting down.");
				shutdown();
			} else {
				static_map = map;
			}
		} else if (type == GLOBAL_COSTMAP)
			static_global_costmap = map;
		else if (type == LOCAL_COSTMAP)
			static_local_costmap = map;
		else
			ROS_FATAL("Unsupported map type : %d", type);
	} else {
		ROS_ERROR("No map received !");
	}
}

void cmd_callback1(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	cmd_callback(map, STATIC_MAP);
}

boost::shared_ptr<nav_msgs::OccupancyGrid> buildOccupancyGrid() {
	static uint32_t seq = 1;
	ROS_DEBUG("Sending map");
	auto grid = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid());
	grid->header.seq = seq ++;
	grid->header.stamp = Time::now();
	grid->header.frame_id = "map";

	grid->info.map_load_time = Time::now();
	grid->info.resolution = 1./static_grid->getResolution();
	grid->info.width = static_grid->getWidth();
	grid->info.height = static_grid->getHeight();
	grid->info.origin = static_map->info.origin;

	const double *griddata = (*static_grid)[0];
	const int area = grid->info.width * grid->info.height;
	const double scalepower = 1./log10(area); // Exponential scale factor
	for (int i = 0; i < area; i ++) {
		double value = min(max(pow(griddata[i], scalepower), 0.), 1.);
		grid->data.push_back((int8_t)(100*(1.-value)));
	}

	return grid;
}

void make_goal(PoseStamped *pose, double x, double y, double yaw) {
	static uint32_t seq = 0;

	pose->header.seq = seq++;
	pose->header.stamp = Time::now();
	pose->header.frame_id = "map";

	pose->pose.position.x = x;
	pose->pose.position.y = y;
	pose->pose.position.z = 0;

	pose->pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

int simulateDetectAt(double x, double y, int sourcex, int sourcey, double dt) {
	static default_random_engine *gen = nullptr;
	if (gen == nullptr) {
		ROS_INFO("Instanciating generator");
		gen = new default_random_engine;
		gen->seed(time(NULL));
	}
	double mean = static_grid->encounterRate(x, y, sourcex, sourcey) * dt;//, entropy = static_grid->entropy();
	poisson_distribution<int> pdist(mean);
	(*gen)();
	int detects = pdist(*gen);
	//ROS_DEBUG("simulateDetectAt(%.1f:%.1f, %d:%d, %.3fms): %d detections", x, y, sourcex, sourcey, dt, detects);
	return detects;
}

uint8_t getCostMapAt(Pose pose, bool local) {
	if (static_local_costmap) {
		auto map = local ? static_local_costmap : static_global_costmap;
		ROS_DEBUG("%s costmap: %s, origin: %f:%f, size: %d:%d, resolved size: %f:%f (res %f)", local ? "local" : "global",
		map->header.frame_id.c_str(), map->info.origin.position.x,map->info.origin.position.y,
		map->info.width, map->info.height,
		map->info.resolution*map->info.width, map->info.resolution*map->info.height, map->info.resolution);
		int gridx = (int)((pose.position.x - map->info.origin.position.x) / map->info.resolution),
		gridy = (int)((pose.position.y - map->info.origin.position.y) / map->info.resolution);
		if (gridx < 0 || gridy < 0 || gridx >= map->info.width || gridy >= map->info.height) {
			if (local)
				return 0u;
			ROS_ERROR("Error: costmap lookup outside of costmap boundaries (%d:%d)", gridx, gridy);
			return 255;
		}
		ROS_DEBUG("Requested relative coords: %.2f:%.2f. Abs coords: %.2f:%.2f. Grid coords : %d:%d. Value : %u",
		pose.position.x, pose.position.y,
		pose.position.x, pose.position.y,
		gridx, gridy, (uint8_t)map->data[gridx + gridy*map->info.width]);
		return (uint8_t)map->data[gridx + gridy*map->info.width];
	}
	ROS_ERROR("Error : no costmap received");
	return 255;
}

bool checkCostmap(Pose pose, bool local) {
	return getCostMapAt(pose, local) == 0; // Don't go anywhere near an obstacle
}

double averageProbability(int centerx, int centery, int radius) {
	int minx = max(centerx-radius, 0),
		maxx = min(centerx+radius, static_grid->getWidth()),
		miny = max(centery-radius, 0),
		maxy = min(centery+radius, static_grid->getHeight());
	double sum = 0, coefsum = 0;
	ROS_INFO("x = [%d:%d], y = [%d:%d]", minx, maxx, miny, maxy);
	for (int y = miny; y < maxy; y ++)
		for (int x = minx; x < maxx; x ++)
			if (hypot(x-centerx, y-centery)<= radius) { // We're in a rectangle, trimming it down to a circle
				double coef = exp(-pow(hypot(x-centerx, y-centery)*2./radius, 2)); // Gaussian scaling
				sum += (*static_grid)[y][x]*coef;
				coefsum += coef;
			}
	return sum / coefsum;
}


struct move {
	infotaxis::Direction dir;
	Pose pose;
	double value;
};

void writeGrid(string filename, double curx, double cury, const int ratio, const int *sourcex, const int *sourcey);

void movesToPoseArray(const vector<struct move> &moves, PoseArray &array) {
	array.poses.clear();
	for (const struct move &m : moves)
		array.poses.push_back(m.pose);
}

void addMoves(double gridx, double gridy, Point tr, double dt, int n, vector<struct move> &moves);

static const double diff = 10, diffrate = 1, windagl = -50 * (M_PI/180), windvel = 1,
	a = 1, dt = 1, resolution = 5.,
	max_stepdist = 5, med_stepdist = 1, min_stepdist = 1/resolution,
	sourcex = -7.8, sourcey = 16.9;

int main (int argc, char *argv[]) {
	init(argc, argv, "infotaxis_node");
	NodeHandle node;
	//string name = this_node::getName();
	NodeHandle priv = NodeHandle("~");
	srand(time(nullptr));

	Subscriber mapsub = node.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(cmd_callback, _1, STATIC_MAP));
	Subscriber localcostmapsub = node.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, boost::bind(cmd_callback, _1, LOCAL_COSTMAP));
	Subscriber globalcostmapsub = node.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, boost::bind(cmd_callback, _1, GLOBAL_COSTMAP));
	bool simulate = true, printimages = true, detect_while_moving = true;

	auto grid_publisher = priv.advertise<nav_msgs::OccupancyGrid>("infotaxis_grid", 1, true);
	auto array_publisher = priv.advertise<PoseArray>("next_steps", 1, true);
	Publisher sourcepos_publisher;
	if (simulate)
	sourcepos_publisher = priv.advertise<PoseStamped>("source_pos", 1, true);

	static_tfl = new tf::TransformListener();

	Rate rate(1);
	int w, h, ttl = 400, min_stepnumber = 24, stepnumber = 4, max_stepnumber = 100;
	bool fast = false;
	//dt = rate.cycleTime().sec + rate.cycleTime().nsec/1000000000.;
	auto currentstate = STATE_DETECTING, nextstate = currentstate;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mbc("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	Point prevPose;
	while(!mbc.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	while (!static_map) {
		rate.sleep();
		spinOnce();
	}
	double originx = static_map->info.origin.position.x,
	originy = static_map->info.origin.position.y;
	// Convert from OccupancyGrid resolution to InfotaxisGrid resolution
	// Note: the former is in m/cell while the latter is in cell/m
	w = (int)(static_map->info.width * static_map->info.resolution * resolution);
	h = (int)(static_map->info.height * static_map->info.resolution * resolution);

	ROS_INFO("Map size : %u:%u * %.3f * %.3f = grid size : %d:%d. Origin offset : %f:%f, source pos : %f:%f, grid pos: %d:%d", static_map->info.width, static_map->info.height, static_map->info.resolution, resolution, w, h, originx, originy, sourcex, sourcey, (int)((sourcex-originx)*resolution), (int)((sourcey-originy)*resolution));

	infotaxis::InfotaxisGrid* grid = new infotaxis::InfotaxisGrid(w, h, diff, diffrate, windvel, windagl, ttl, a, resolution, !fast);
	static_grid = boost::shared_ptr<infotaxis::InfotaxisGrid>(grid);

	PoseStamped source_pose;
	source_pose.header.frame_id = "/map";
	source_pose.pose.position.x = sourcex;
	source_pose.pose.position.y = sourcey;
	source_pose.pose.position.z = 0;
	source_pose.pose.orientation = tf::createQuaternionMsgFromYaw(windagl);

	system("rm -f images/*.png"); // clear iteration frames
	int seq = 0, imageseq = 0;
	if (printimages) {
		int gridsourcex = (int)((sourcex-originx)*resolution), gridsourcey = (int)((sourcey-originy)*resolution);
		writeGrid("images/meanfield.png", 0, 0, 1, &gridsourcex, &gridsourcey);
	}

	boost::shared_ptr<nav_msgs::OccupancyGrid> occupancyGrid = buildOccupancyGrid();
	vector<struct move> directions, prev_directions;
	char* filename = new char[64];

	while(ok()) {
		ROS_INFO("State : %s (%d)", STATE_NAMES[nextstate], nextstate);
		spinOnce();
		try {
			Point basePoint;
			{
				PoseStamped stamp, stamp2;
				stamp.header.frame_id = "/base_link";
				stamp.pose.position.x = stamp.pose.position.y = stamp.pose.position.z = 0;
				stamp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
				static_tfl->transformPose("/map", stamp, stamp2);
				basePoint = stamp2.pose.position;
			}
			double gridx = (basePoint.x-originx) * resolution,
			gridy = (basePoint.y-originy) * resolution;
			ROS_DEBUG("Position : %.3f:%.3f, grid position : %d:%d", basePoint.x, basePoint.y,
			(int)gridx, (int)gridy);
			switch(currentstate) {
				case STATE_PLANNING: {

					//directions.push_back({0.,0.});
					if (directions.size() < min_stepnumber)
						addMoves(gridx, gridy, basePoint, dt, max(stepnumber, min_stepnumber - (int)directions.size()), directions);
					else {
						if (directions.size() == 0) {
							ROS_ERROR("Error: no movement possible. Aborting");
							nextstate = STATE_EXIT;
						} else {
							auto iter = max_element(directions.begin(), directions.end(), [](struct move m1, struct move m2) {return m1.value < m2.value;});
							auto optimove = iter->dir;
							ROS_INFO("Optimal move : %.3f:%.3f, %.3fm:%.3fm on the map (angle %d)", optimove.dx, optimove.dy,
							optimove.dx/resolution, optimove.dy/resolution, (int)(atan2(optimove.dy, optimove.dx)*180/M_PI));

							if (abs(optimove.dx) + abs(optimove.dy) > 0.0) {
								prevPose = basePoint;
								make_goal(&goal.target_pose, basePoint.x + optimove.dx/resolution, basePoint.y + optimove.dy/resolution, atan2(optimove.dy, optimove.dx));
								ROS_INFO("Sending goal");
								mbc.sendGoal(goal);
								nextstate = STATE_MOVING;
								directions.erase(iter);
								prev_directions = directions;
								directions.clear();
							} else {
								nextstate = STATE_EXIT;
								//nextstate = STATE_DETECTING;
							}
						}
					}
				}
				break;
				case STATE_MOVING:
				ROS_DEBUG("State : %s", mbc.getState().getText());
				if (mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					if (simulate && hypot(gridx - sourcex, gridy - sourcey) < .5) {
						ROS_INFO("Source found !");
						nextstate = STATE_EXIT;
					} else {
						ROS_INFO("Destination reached");
						nextstate = STATE_DETECTING;
					}
				} else if (mbc.getState().isDone()) {
					double	goaldist = hypot(goal.target_pose.pose.position.x - basePoint.x,
											goal.target_pose.pose.position.y - basePoint.y),
							sourcedist = hypot(prevPose.x - basePoint.x,
												prevPose.y - basePoint.y);
					ROS_ERROR("move_base returned failure state : %s", mbc.getState().getText().c_str());
					if (goaldist < sourcedist) {
						ROS_ERROR("Closer to destination than source : using new destinations");
					} else {
						ROS_ERROR("Closer to source than destination : restoring old goals");
						directions = prev_directions;
					}
					nextstate = STATE_PLANNING;
				} else {
					addMoves(gridx, gridy, goal.target_pose.pose.position, dt, stepnumber, directions);
				}
				if (!detect_while_moving)
					break;
				case STATE_DETECTING: {
					int detections = simulateDetectAt(gridx, gridy, (int)((sourcex-originx)*resolution), (int)((sourcey-originy)*resolution), dt);
					grid->updateProbas(gridx, gridy, detections, dt);
					if (detections > 0)
						ROS_INFO("Detections : %d, new entropy: %f", detections, grid->entropy());
					if (currentstate == STATE_DETECTING && printimages) {
						// don't write images if detecting while moving, too frequent
						writeGrid(filename, gridx, gridy, 1, nullptr, nullptr);
						nextstate = STATE_PLANNING;
					}
					sprintf(filename, "images/iteration%04d.png", imageseq++);
					writeGrid(filename, gridx, gridy, 1, nullptr, nullptr);
					occupancyGrid = buildOccupancyGrid();
				}
				break;
			}
		} catch (tf::TransformException &ex){
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		PoseArray array;
		array.header.frame_id = "/map";
		source_pose.header.seq = array.header.seq = seq ++;
		source_pose.header.stamp = array.header.stamp = Time::now();
		movesToPoseArray(directions, array);
		array_publisher.publish(array);
		if (simulate)
			sourcepos_publisher.publish(source_pose);
		grid_publisher.publish(occupancyGrid);
		rate.sleep();
		if (nextstate != currentstate) {
			currentstate = nextstate;
		}
		if (nextstate == STATE_EXIT)
			shutdown();
	}

	delete[] filename;
	delete static_tfl;
	return 0;
}

void writeGrid(string filename, double curx, double cury, const int ratio, const int *sourcex, const int *sourcey) {
	const int imgw = static_grid->getWidth() * ratio, imgh = static_grid->getHeight() * ratio;
	png::image<png::rgb_pixel> image((size_t) imgw, (size_t) imgh);

	if (sourcex == NULL || sourcey == NULL)
	static_grid->writeProbabilityField(image, ratio);
	else
	static_grid->writeMeanStationaryField(image, *sourcex, *sourcey, ratio);
	image.write(filename);
}

void addMoves(double gridx, double gridy, Point ori, double dt, int n, vector<struct move> &moves) {
	double interest = pow(averageProbability(gridx, gridy, 25), 1./25.), angle = rand() * 2*M_PI / RAND_MAX,
		mindist = (interest) * med_stepdist + (1.-interest) * max_stepdist,
		maxdist = (interest) * min_stepdist + (1.-interest) * med_stepdist;
	ROS_DEBUG("Interest: %f. dist range = [%.2f:%.2f]", interest, mindist, maxdist);

	for (int i = 0; i < n; i ++) {
		double dist = (rand()/(double)RAND_MAX) * (maxdist-mindist) + mindist,
			x = cos(angle) * dist, y = sin(angle) * dist;
		Pose pose;
		pose.position.x = ori.x + x;
		pose.position.y = ori.y + y;
		pose.position.z = 0;
		pose.orientation = tf::createQuaternionMsgFromYaw(angle);
		if (!isfinite(pose.position.x) || !isfinite(pose.position.y)) {
			ROS_FATAL("Error: %f:%f position generated during planning",
				pose.position.x, pose.position.y);
			shutdown();
		}
		bool loccheck = checkCostmap(pose, true), globcheck = checkCostmap(pose, false);
		if (loccheck && globcheck) {
			infotaxis::Direction direc = {x * resolution, y * resolution};
			moves.push_back((struct move){direc, pose, static_grid->getMoveValue(gridx, gridy, dt, direc)});
		} else
			ROS_DEBUG("Move at %.1f obstructed (%s)", angle*180/M_PI, globcheck ? "local" : (loccheck ? "global" : "both"));
		angle += 2*M_PI/n;
	}
}