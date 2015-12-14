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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <infotaxis.hpp>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static double rescale_ratio = NAN;
static int tot_xoff = 0, tot_yoff = 0;
static auto s_map = 		nav_msgs::OccupancyGrid::ConstPtr();
static auto s_grid = 		boost::shared_ptr<infotaxis::InfotaxisGrid>();
static auto s_local_costmap  =	nav_msgs::OccupancyGrid::ConstPtr();
static auto s_global_costmap = nav_msgs::OccupancyGrid::ConstPtr();
static tf::TransformListener 	*s_tfl;
//static Publisher grid_publisher;

static double resolution, max_stepdist, med_stepdist, min_stepdist;

enum RobotStates {
	STATE_EXIT = 0,
	STATE_DETECTING = 1,
	STATE_PLANNING = 2,
	STATE_MOVING = 3
};

static const char* STATE_NAMES[] = {"EXIT", "DETECTING", "PLANNING", "MOVING"};

enum class SubscribedMapTypes {
	STATIC_MAP,
	LOCAL_COSTMAP,
	GLOBAL_COSTMAP
};

void cmd_callback(const nav_msgs::OccupancyGrid::ConstPtr& map, SubscribedMapTypes type) {
	ROS_DEBUG("Received %smap", type == SubscribedMapTypes::STATIC_MAP ? "" : "cost");
	if (map) {
		ROS_DEBUG("seq:%u, res:%f, size:%ux%u, pos:%f:%f:%f\n", map->header.seq, map->info.resolution, map->info.width, map->info.height, map->info.origin.position.x, map->info.origin.position.y, map->info.origin.position.z);
	if (type == SubscribedMapTypes::STATIC_MAP) {
			if (s_map && (s_map->info.width != map->info.width || s_map->info.height != map->info.height)) {
				int w = ceil(map->info.width * map->info.resolution * resolution);
				int h = ceil(map->info.height * map->info.resolution * resolution);
				int xoff = (int)((s_map->info.origin.position.x-map->info.origin.position.x) * resolution);
				int yoff = (int)((s_map->info.origin.position.y-map->info.origin.position.y) * resolution);
				tot_xoff += xoff;
				tot_yoff += yoff;
				ROS_INFO("Map updated with different dimensions : Resizing from %d:%d to %d:%d + %d:%d (rescale dimension %d)...",
					s_grid->getWidth(), s_grid->getHeight(), w,h,xoff,yoff, (int)(max(w, h)/rescale_ratio));
				ROS_INFO("%f - %f, %f - %f, %f", s_map->info.origin.position.x-map->info.origin.position.x,
					s_map->info.origin.position.y-map->info.origin.position.y, resolution);
				s_grid = boost::shared_ptr<infotaxis::InfotaxisGrid>(s_grid->resize(
					w, h, xoff, yoff, infotaxis::ResizeMethod::RESCALE,
					(int)(max(w, h)/rescale_ratio)));
				ROS_INFO("Done!");
			}
			s_map = map;
		} else if (type == SubscribedMapTypes::GLOBAL_COSTMAP)
			s_global_costmap = map;
		else if (type == SubscribedMapTypes::LOCAL_COSTMAP)
			s_local_costmap = map;
		else
			ROS_FATAL("Unsupported map type : %d", type);
	} else {
		ROS_ERROR("No map received !");
	}
}

void cmd_callback1(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	cmd_callback(map, SubscribedMapTypes::STATIC_MAP);
}

boost::shared_ptr<nav_msgs::OccupancyGrid> buildOccupancyGrid() {
	static uint32_t seq = 1;
	ROS_DEBUG("Sending map");
	auto grid = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid());
	grid->header.seq = seq;
	grid->header.stamp = Time::now();
	grid->header.frame_id = "map";

	grid->info.map_load_time = Time::now();
	grid->info.resolution = 1./s_grid->getResolution();
	grid->info.width = s_grid->getWidth();
	grid->info.height = s_grid->getHeight();
	grid->info.origin = s_map->info.origin;

	const double *griddata = (*s_grid)[0];
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
	double mean = s_grid->encounterRate(x, y, sourcex, sourcey) * dt;//, entropy = s_grid->entropy();
	poisson_distribution<int> pdist(mean);
	(*gen)();
	int detects = pdist(*gen);
	//ROS_DEBUG("simulateDetectAt(%.1f:%.1f, %d:%d, %.3fms): %d detections", x, y, sourcex, sourcey, dt, detects);
	return detects;
}

int8_t getCostMapAt(Pose pose, SubscribedMapTypes type) {
	auto map = type == 	SubscribedMapTypes::STATIC_MAP 	? 	s_map :
				type == SubscribedMapTypes::LOCAL_COSTMAP ? s_local_costmap :
										s_global_costmap;
	if (map) {
		int gridx = (int)((pose.position.x - map->info.origin.position.x) / map->info.resolution),
		gridy = (int)((pose.position.y - map->info.origin.position.y) / map->info.resolution);
		if (gridx < 0 || gridy < 0 || gridx >= map->info.width || gridy >= map->info.height) {
			if (type == SubscribedMapTypes::LOCAL_COSTMAP)
				return 0;
			return -1;
		}
		ROS_DEBUG("Requested relative coords: %.2f:%.2f. Abs coords: %.2f:%.2f. Grid coords : %d:%d. Value : %u",
		pose.position.x, pose.position.y,
		pose.position.x, pose.position.y,
		gridx, gridy, (uint8_t)map->data[gridx + gridy*map->info.width]);
		return map->data[gridx + gridy*map->info.width];
	}
	return -1;
}

bool checkMaps(Pose pose) {
	return	getCostMapAt(pose, SubscribedMapTypes::STATIC_MAP) == 0 &&
			getCostMapAt(pose, SubscribedMapTypes::LOCAL_COSTMAP) == 0 &&
			getCostMapAt(pose, SubscribedMapTypes::GLOBAL_COSTMAP) == 0; // Don't go anywhere near an obstacle
}

double averageProbability(int centerx, int centery, int radius) {
	int minx = max(centerx-radius, 0),
		maxx = min(centerx+radius, s_grid->getWidth()),
		miny = max(centery-radius, 0),
		maxy = min(centery+radius, s_grid->getHeight());
	double sum = 0, coefsum = 0;
	for (int y = miny; y < maxy; y ++)
		for (int x = minx; x < maxx; x ++)
			if (hypot(x-centerx, y-centery)<= radius) { // We're in a rectangle, trimming it down to a circle
				double coef = exp(-pow(hypot(x-centerx, y-centery)*2./radius, 2)); // Gaussian scaling
				sum += (*s_grid)[y][x]*coef;
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

int main(int argc, char *argv[]) {
	init(argc, argv, "infotaxis_node");
	NodeHandle node;
	//string name = this_node::getName();
	NodeHandle priv = NodeHandle("~");
	srand(time(nullptr));

	double diff, diffrate, windagl, windvel, a, dt, goalTolerance,
	 	sourcex, sourcey;
	bool simulate, printimages, printcsv, detect_while_moving, fast, publish_backtrace;
	int w, h, ttl, min_stepnumber, stepnumber, max_stepnumber,
		movingImageWriteFrequency;
	string imageDir, csvDir, logfilename;

	priv.param("gas_diffusivity", diff, 10.);
	priv.param("gas_diff_rate", diffrate, 1.);
	priv.param("gas_ttl", ttl, 400);
	priv.param("wind_angle", windagl, 0.);
	priv.param("wind_velocity", windvel, 1.);
	priv.param("sensor_radius", a, 1.);
	priv.param("delta_t", dt, 1.);
	priv.param("resolution", resolution, 5.);
	priv.param("fast_infotaxis", fast, false);
	priv.param("max_stepdist", max_stepdist, 10.);
	priv.param("med_stepdist", med_stepdist, 1.);
	priv.param("min_stepdist", min_stepdist, 1./resolution);
	priv.param("min_stepnumber", min_stepnumber, 24);
	priv.param("stepnumber", stepnumber, 4);
	priv.param("max_stepnumber", max_stepnumber, 200);
	priv.param("rescale_ratio", rescale_ratio, 16.0);
	priv.param("logfile", logfilename, string("run.log"));

	priv.param("publish_backtrace", publish_backtrace, true);
	priv.param("print_images", printimages, false);
	priv.param("image_dir", imageDir, string("pictures"));
	priv.param("print_csv", printcsv, false);
	priv.param("csv_dir", csvDir, string("csvs"));
	priv.param("detect_while_moving", detect_while_moving, true);
	priv.param("moving_image_write_frequency", movingImageWriteFrequency, 4);
	priv.param("goal_tolerance", goalTolerance, .2);
	priv.param("source_x", sourcex, 0.);
	priv.param("source_y", sourcey, 0.);
	priv.param("simulate_source", simulate, false);

	auto logfile = logfilename.length() > 0 ? new ofstream(logfilename) : NULL;

	Rate rate(1);
	{
		double rate_time;
		priv.param("run_rate", rate_time, 1.);
		rate = Rate(rate_time);
	}

	Subscriber mapsub = node.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(cmd_callback, _1, SubscribedMapTypes::STATIC_MAP));
	Subscriber localcostmapsub = node.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, boost::bind(cmd_callback, _1, SubscribedMapTypes::LOCAL_COSTMAP));
	Subscriber globalcostmapsub = node.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, boost::bind(cmd_callback, _1, SubscribedMapTypes::GLOBAL_COSTMAP));

	auto grid_publisher = priv.advertise<nav_msgs::OccupancyGrid>("infotaxis_grid", 1, true);
	auto array_publisher = priv.advertise<PoseArray>("next_steps", 1, true);
	Publisher sourcepos_publisher, backtrace_publisher;
	if (simulate)
		sourcepos_publisher = priv.advertise<PoseStamped>("source_pos", 1, true);
	else {
		ROS_FATAL("Error: Actual sensor reading not implemented yet.");
		return 1;
	}
	if (publish_backtrace)
		backtrace_publisher = priv.advertise<nav_msgs::Path>("backtrace", 1, true);

	s_tfl = new tf::TransformListener();

	//dt = rate.cycleTime().sec + rate.cycleTime().nsec/1000000000.;
	auto currentstate = STATE_DETECTING, nextstate = currentstate;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mbc("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	Point prevPose;
	while(!mbc.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	while (!s_map || !s_local_costmap || !s_global_costmap) {
		rate.sleep();
		spinOnce();
	}
	// Convert from OccupancyGrid resolution to InfotaxisGrid resolution
	// Note: the former is in m/cell while the latter is in cell/m
	w = (int)(s_map->info.width * s_map->info.resolution * resolution);
	h = (int)(s_map->info.height * s_map->info.resolution * resolution);

	if (logfile != NULL)
		*logfile << w <<" "<< h <<" "<< diff <<" "<< diffrate <<" "<< windvel <<" "<< windagl <<" "<< ttl
			<<" "<< a <<" "<< resolution <<" "<< fast << dt << endl << endl;

	ROS_INFO("Map size : %u:%u * %.3f * %.3f = grid size : %d:%d. Origin offset : %f:%f, source pos : %f:%f, grid pos: %d:%d", s_map->info.width, s_map->info.height, s_map->info.resolution, resolution, w, h, s_map->info.origin.position.x, s_map->info.origin.position.y, sourcex, sourcey, (int)((sourcex-s_map->info.origin.position.x)*resolution), (int)((sourcey-s_map->info.origin.position.y)*resolution));

	ROS_INFO("Global costmap size : %u:%u * %.3f",
	s_global_costmap->info.width, s_global_costmap->info.height, s_global_costmap->info.resolution);

	s_grid = boost::shared_ptr<infotaxis::InfotaxisGrid>(new infotaxis::InfotaxisGrid(w, h, diff, diffrate, windvel, windagl, ttl, a, resolution, !fast));

	PoseStamped source_pose;
	source_pose.header.frame_id = "/map";
	source_pose.pose.position.x = sourcex;
	source_pose.pose.position.y = sourcey;
	source_pose.pose.position.z = 0;
	source_pose.pose.orientation = tf::createQuaternionMsgFromYaw(windagl);

	int seq = 0, detectseq = 0;

	boost::shared_ptr<nav_msgs::OccupancyGrid> occupancyGrid = buildOccupancyGrid();
	nav_msgs::Path backtrace;
	backtrace.header.frame_id = "/map";

	vector<struct move> directions, prev_directions;
	char* filename = new char[64];
	if (printimages) {
		sprintf(filename, "rm -f %s/*.png", imageDir.c_str());
		system(filename); // clear iteration frames
		sprintf(filename, "%s/meanfield.png", imageDir.c_str());
		int gridsourcex = (int)((sourcex-s_map->info.origin.position.x)*resolution), gridsourcey = (int)((sourcey-s_map->info.origin.position.y)*resolution);
		writeGrid("pictures/meanfield.png", 0, 0, 1, &gridsourcex, &gridsourcey);
		ROS_INFO("Printing images to %s", imageDir.c_str());
	}
	if (printcsv) {
		sprintf(filename, "rm -f %s/*.csv", csvDir.c_str());
		system(filename); // clear iteration frames
		ROS_INFO("Printing CSVs to %s", csvDir.c_str());
	}

	while(ok()) {
		ROS_DEBUG("State : %s (%d)", STATE_NAMES[nextstate], nextstate);
		spinOnce();
		try {
			PoseStamped basePose;
			Point basePoint;
			{
				PoseStamped stamp, stamp2;
				stamp.header.frame_id = "/base_link";
				stamp.pose.position.x = stamp.pose.position.y = stamp.pose.position.z = 0;
				stamp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
				s_tfl->transformPose("/map", stamp, basePose);
				basePoint = basePose.pose.position;
			}
			double gridx = (basePoint.x-s_map->info.origin.position.x) * resolution,
			gridy = (basePoint.y-s_map->info.origin.position.y) * resolution;
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
				case STATE_MOVING: {
					ROS_DEBUG("State : %s", mbc.getState().getText());
					if (mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
							hypot(goal.target_pose.pose.position.x - basePoint.x,
							goal.target_pose.pose.position.y - basePoint.y) <= goalTolerance) {
								// Arrived at goal or close enough
						if (simulate && hypot(basePoint.x - sourcex, basePoint.y - sourcey) < 1.) {
							ROS_INFO("Source found !");
							nextstate = STATE_EXIT;
						} else {
							ROS_INFO("Destination reached. Distance to source : %.3f", hypot(basePoint.x - sourcex, basePoint.y - sourcey));
							nextstate = (detect_while_moving) ? STATE_PLANNING : STATE_DETECTING;
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
					} else if (directions.size() < max_stepnumber) {
						addMoves(gridx, gridy, goal.target_pose.pose.position, dt, stepnumber, directions);
					}
				}
				if (!detect_while_moving) // If true, fall through to DETECTING
					break;
				case STATE_DETECTING: {
					int detections = simulateDetectAt(gridx, gridy, (int)((sourcex-s_map->info.origin.position.x)*resolution), (int)((sourcey-s_map->info.origin.position.y)*resolution), dt);
					s_grid->updateProbas(gridx, gridy, detections, dt);
					if (logfile != NULL)
						*logfile << s_grid->getWidth() <<" "<< s_grid->getHeight()<<" "<< tot_xoff <<" "<< tot_yoff
						<<" "<< gridx <<" "<< gridy <<" "<< s_grid->entropy() <<" "<< detections << endl;
					backtrace.poses.push_back(basePose);
					if (detections > 0)
						ROS_INFO("Detections : %d, new entropy: %f", detections, s_grid->entropy());
					if (printimages && (currentstate == STATE_DETECTING ||
							// while moving, only write images at the given frequency
							(currentstate == STATE_MOVING && (seq % movingImageWriteFrequency) == 0))) {
						sprintf(filename, "%s/iteration%04d.png", imageDir.c_str(), detectseq);
						writeGrid(filename, gridx, gridy, 1, nullptr, nullptr);
					}
					if (printcsv) {
						sprintf(filename, "%s/iteration%04d.csv", csvDir.c_str(), detectseq);
						s_grid->toCSV(filename);
					}
					if (currentstate == STATE_DETECTING)
						nextstate = STATE_PLANNING;
					occupancyGrid = buildOccupancyGrid();
					detectseq ++;
				}
				break;
			}
		} catch (tf::TransformException &ex){
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		PoseArray array;
		array.header.frame_id = "/map";
		array.header.seq = seq;
		array.header.stamp = Time::now();
		source_pose.header = backtrace.header = array.header;
		movesToPoseArray(directions, array);
		array_publisher.publish(array);

		if (publish_backtrace)
			backtrace_publisher.publish(backtrace);

		if (simulate)
			sourcepos_publisher.publish(source_pose);

		grid_publisher.publish(occupancyGrid);
		rate.sleep();
		if (nextstate != currentstate) {
			currentstate = nextstate;
		}
		if (nextstate == STATE_EXIT)
			shutdown();
		seq ++;
	}


	if (logfile != NULL) {
		logfile->close();
		delete logfile;
	}
	delete[] filename;
	delete s_tfl;
	ROS_INFO("Exitting gracefully");
	return 0;
}

void writeGrid(string filename, double curx, double cury, const int ratio, const int *sourcex, const int *sourcey) {
	const int imgw = s_grid->getWidth() * ratio, imgh = s_grid->getHeight() * ratio;
	png::image<png::rgb_pixel> image((size_t) imgw, (size_t) imgh);

	if (sourcex == NULL || sourcey == NULL)
		s_grid->writeProbabilityField(image, ratio);
	else
		s_grid->writeMeanStationaryField(image, *sourcex, *sourcey, ratio);
	image.write(filename);
}

void addMoves(double gridx, double gridy, Point ori, double dt, int n, vector<struct move> &moves) {
	double interest = pow(averageProbability(gridx, gridy, 25), 1./25.), angle = rand() * 2*M_PI / RAND_MAX,
		mindist = (interest) * med_stepdist + (1.-interest) * max_stepdist,
		maxdist = (interest) * min_stepdist + (1.-interest) * med_stepdist;
	ROS_DEBUG("Interest: %f. dist range = [%.2f:%.2f]", interest, mindist, maxdist);

	const int GIVEUP = 50;
	int tries = 0, targetSize = moves.size() + n;
	auto mapinfo = s_global_costmap->info;
	while (moves.size() < targetSize && tries < GIVEUP) {
		tries ++;
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
		if (checkMaps(pose)) {
			infotaxis::Direction direc = {x * resolution, y * resolution};
			moves.push_back((struct move){direc, pose, s_grid->getMoveValue(gridx, gridy, dt, direc)});
			angle += 2*M_PI/n;
			tries = 0;
		} else {
			ROS_DEBUG("Move at %.1f obstructed", angle*180/M_PI);
			angle += rand()*(M_PI / 2) / RAND_MAX;
		}
	}
	if (moves.size() == 0)
		throw runtime_error("Couldn't calculate any direction");
}
