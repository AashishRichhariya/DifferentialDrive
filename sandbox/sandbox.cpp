#include "aprilvideointerface.h"
#include <unistd.h>
#include "pathplanners.h"
#include "controllers.h"
// For Arduino: serial port access class
#include "Serial.h"
using namespace std;
using namespace cv;

struct bot_config{
  PathPlannerGrid plan;
  PurePursuitController control;//constructor called, thus must have a default constructor with no arguments
  int id;
  robot_pose pose;
  //using intializer list allows intializing variables with non trivial contructor
  //assignment would not help if there is no default contructor with no arguments
  bot_config(int cx,int cy, int thresh,vector<vector<nd> > &tp, double a,double b,double c, int d,int e,int f,bool g):plan(PathPlannerGrid(cx,cy,thresh,tp)),control(PurePursuitController(a,b,c,d,e,f,g)){
    id = -1;//don't forget to set the id
    //below line would first call plan PathPlannerGrid constructor with no argument and then replace lhs variables with rhs ones
    //plan = PathPlannerGrid(cx,cy,thresh,tp);
  }
  void init(){
    plan.start_grid_x = plan.start_grid_y = -1;
    plan.robot_id = -1;
  }
};

int check_deadlock(vector<bot_config> &bots, int index)
{
  cout<<"\nChecking deadlock presence:\n"<<endl;
  for(int i = 0; i < bots.size(); i++)
  {
    bots[i].plan.deadlock_check_counter = 0;
  }
  int clear_flag = 0;
  int target_cell_bot_id = -1;
  while(!clear_flag)
  {
    cout<<"index: "<<index<<endl;
    int r = bots[index].plan.target_grid_cell.first;
    int c = bots[index].plan.target_grid_cell.second;
    cout<<"r,c :"<<r<<" "<<c<<endl;
    if(bots[index].plan.world_grid[r][c].bot_presence.first == 1 && bots[index].plan.world_grid[r][c].bot_presence.second != bots[index].plan.robot_tag_id)
    {
      target_cell_bot_id = bots[index].plan.world_grid[r][c].bot_presence.second;
      bots[target_cell_bot_id].plan.deadlock_check_counter++;
      if(bots[target_cell_bot_id].plan.deadlock_check_counter > 1)
      {
        break;
      }
      else if(bots[target_cell_bot_id].plan.status == 2)// to check if the said target bot has covered all its point and is in no position to move
      {
        break;
      }
      index = target_cell_bot_id;
      continue;
    }
    else
    {
      clear_flag = 1;
    }

  }
  if(clear_flag == 1)
  {
    return -1;
  }
  else
  {
    return target_cell_bot_id;
  }

}

void check_collision_possibility(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &planners, vector<bot_config> &bots, pair<int,int> &wheel_velocities, int i)
{
  if(bots[i].plan.next_target_index != bots[i].plan.path_points.size()) //for collision avoidance
  {
    int c = (bots[i].plan.pixel_path_points[bots[i].plan.next_target_index].first)/(bots[i].plan.cell_size_x);
    int r = (bots[i].plan.pixel_path_points[bots[i].plan.next_target_index].second)/(bots[i].plan.cell_size_y);
    bots[i].plan.target_grid_cell = make_pair(r, c);
    if(bots[i].plan.world_grid[r][c].bot_presence.first == 1 && bots[i].plan.world_grid[r][c].bot_presence.second != bots[i].plan.robot_tag_id)
    {
      wheel_velocities = make_pair(0,0);
      cout<<"A Bot is present in target cell!\n";
      cout<<"r,c: "<<r<<" "<<c<<endl;
      cout<<"Present robot tag id: "<<bots[i].plan.world_grid[r][c].bot_presence.second<<endl;
      int deadlocked_bot = check_deadlock(bots, i);
      if(deadlocked_bot != -1)
      {
      cout<<"\n******\n";
      cout<<"Deadlock Detected!"<<endl;
      bots[deadlocked_bot].plan.DeadlockReplan(testbed, planners);
      cout<<"Path Replanned!"<<endl;
      cout<<"******\n\n";
      }
    }
  }
}



int main(int argc, char* argv[]) {
  AprilInterfaceAndVideoCapture testbed;
  testbed.parseOptions(argc, argv);
  testbed.setup();
  if (!testbed.isVideo()) {
    cout << "Processing image: option is not supported" << endl;
    testbed.loadImages();
    return 0;
  }
  cout << "Processing video" << endl;
  testbed.setupVideo();
  int frame = 0;
  int first_iter = 1;
  double last_t = tic();
  const char *windowName = "What do you see?";
  cv::namedWindow(windowName,WINDOW_NORMAL);
  /*vector<Serial> s_transmit(2);
  ostringstream sout;
  if(testbed.m_arduino){
    for(int i = 0;i<s_transmit.size();i++){
      sout.str("");
      sout.clear();
      sout<<"/dev/ttyUSB"<<i;
      s_transmit[i].open(sout.str(),9600);
    }
  }*/
  Serial s_transmit;
  s_transmit.open("/dev/ttyUSB0", 9600);
  cv::Mat image;
  cv::Mat image_gray;
  //make sure that lookahead always contain atleast the next path point
  //if not then the next point to the closest would automatically become target
  //PurePursuitController controller(40.0,2.0,14.5,70,70,128,false);
  //PurePursuitController controller(20.0,2.0,14.5,70,70,128,true);
  //PathPlannerUser path_planner(&testbed);
  //setMouseCallback(windowName, path_planner.CallBackFunc, &path_planner);
  int robotCount;
  int max_robots = 5;
  int origin_tag_id = 0;//always 0
  //tag id should also not go beyond max_robots
  vector<vector<nd> > tp;//a map that would be shared among all
  vector<bot_config> bots(max_robots,bot_config(60,60,120,tp,40.0,2.3,14.5,75,75,128,false));
  vector<PathPlannerGrid> planners(max_robots,PathPlannerGrid(tp));

  int algo_select;
  cout<<"\nWhat Algo do you want to compare?\n" 
  "1: BSA-CM (Basic)\n" 
  "2: BSA-CM with updated Backtrack Search\n" 
  "3: Boustrophedon Motion With Updated Bactrack Search\n"
  "4: FAST\n"
  "\nEnter here: ";
  cin>>algo_select;

  while (true){
    for(int i = 0;i<max_robots;i++){
      bots[i].init();
      bots[i].id = i;//0 is saved for origin
    }
    robotCount = 0;
    testbed.m_cap >> image;
    //image = imread("tagimage.jpg");

    testbed.processImage(image, image_gray);//tags extracted and stored in class variable
    int n = testbed.detections.size();
    for(int i = 0;i<bots.size();i++){
      bots[i].plan.robot_tag_id = i;
    }
    for(int i = 0;i<n;i++){
      bots[testbed.detections[i].id].plan.robot_id = i; //robot_id is the index in detections the tag is detected at
      if(testbed.detections[i].id == origin_tag_id){//plane extracted
        bots[testbed.detections[i].id].plan.robot_id = i;
        testbed.extractPlane(i);
        break;
      } 
    }
    if(bots[origin_tag_id].plan.robot_id<0)
      continue;//can't find the origin tag to extract plane
    for(int i = 0;i<n;i++){
      if(testbed.detections[i].id != origin_tag_id){//robot or goal
        if(robotCount>=10){
          cout<<"too many robots found"<<endl;
          break;
        }
        robotCount++;
        testbed.findRobotPose(i,bots[testbed.detections[i].id].pose);//i is the index in detections for which to find pose
      }
    }

    //all robots must be detected(in frame) when overlay grid is called else some regions on which a robot is 
    //present(but not detected) would be considered an obstacle
    //no two robots must be present in the same grid cell(result is undefined)
    if(first_iter){
      first_iter = 0;
      bots[0].plan.overlayGrid(testbed.detections,image_gray);//overlay grid completely reintialize the grid, we have to call it once at the beginning only when all robots first seen simultaneously(the surrounding is assumed to be static) not every iteration
      for(int i = 1;i<bots.size();i++){
        bots[i].plan.rcells = bots[0].plan.rcells;
        bots[i].plan.ccells = bots[0].plan.ccells;
      }
    }

    //the planners[i] should be redefined every iteration as the stack and bt points change 
    //this is inefficient, should look for alternates
    for(int i = 0;i<bots.size();i++){
      //for bot 0, the origin and robot index would be the same
      bots[i].plan.origin_id = bots[0].plan.robot_id;//set origin index of every path planner which is the index of tag 0 in detections vector given by RHS
      planners[i] = bots[i].plan;
    }

    for(int i = 1;i<bots.size();i++){
      cout<<"planning for id "<<i<<endl;
      switch(algo_select)
      {
      case 1: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners); break;
      case 2: bots[i].plan.BSACoverageWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
      case 3: bots[i].plan.BoustrophedonMotionWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
      case 4: bots[i].plan.FAST(testbed,bots[i].pose, 2.5,planners); break;
      default: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners);   
      }   
    }

    //if(!path_planner.total_points){//no path algorithm ever run before, total_points become -1 if no path exists from pos to goal
      //path_planner.robot_id = tag_id_index_map[robot_id];
      //path_planner.goal_id = tag_id_index_map[goal_id];
      //path_planner.origin_id = tag_id_index_map[origin_id];
      //path_planner.overlayGrid(testbed.detections,image_gray);
      //if(path_planner.origin_id>=0 && path_planner.robot_id>=0){
        //path_planner.findCoverageGlobalNeighborPreference(testbed);
        //path_planner.findCoverageLocalNeighborPreference(testbed,robots[pose_id_index_map[robot_id]]);
        //path_planner.BSACoverage(testbed,robots[pose_id_index_map[robot_id]]);
        //if(path_planner.goal_id>=0)
          //path_planner.findshortest(testbed);
      //}
    //}
    

    if(testbed.m_arduino){
      pair<int,int> wheel_velocities;
      for(int i = 1;i<bots.size();i++){//0 is for origin
        //if (i==1)continue;

        int next_point_index_in_path=0; //for collision avoidance

        wheel_velocities = bots[i].control.computeStimuli(bots[i].pose,bots[i].plan.path_points, next_point_index_in_path);//for nonexistent robots, path_points vector would be empty thus preventing the controller to have any effect
          
          bots[i].plan.next_target_index = next_point_index_in_path;
          check_collision_possibility(testbed, planners, bots, wheel_velocities, i);
          
          
         /* s_transmit[i-1].print((unsigned char)(bots[i].id));
          cout<<"sending velocity for bot "<<bots[i].id<<endl;
          s_transmit[i-1].print((unsigned char)(128+wheel_velocities.first));
          s_transmit[i-1].print((unsigned char)(128+wheel_velocities.second));
        */
          s_transmit.print((unsigned char)(bots[i].id));
          cout<<"sending velocity for bot "<<bots[i].id<<endl;
          s_transmit.print((unsigned char)(128+wheel_velocities.first));
          s_transmit.print((unsigned char)(128+wheel_velocities.second));
          cout<<"sent velocities "<<wheel_velocities.first<<" "<<wheel_velocities.second<<endl;

          //strangely when I send multiple values to different robots in this loop, the robot always move straight irrespective of the value sent
          //break;
      }
    }
    if(testbed.m_draw){
      for(int i = 0;i<n;i++){
        testbed.detections[i].draw(image);
      }
      bots[origin_tag_id].plan.drawGrid(image);
      for(int i = 1;i<bots.size();i++){
        bots[i].plan.drawPath(image);
      }
      //add a next point circle draw for visualisation
      //add a only shortest path invocation drawing function in pathplanners
      //correct next point by index to consider reach radius to determine the next point
      imshow(windowName,image);
    }
    // print out the frame rate at which image frames are being processed
    frame++;
    if (frame % 10 == 0) {
      double t = tic();
      cout << "  " << 10./(t-last_t) << " fps" << endl;
      last_t = t;
    }
    if (cv::waitKey(10) == 27){
      if(testbed.m_arduino){
        for(int i = 1;i<bots.size();i++){//0 is for origin
          //sif (i==1)continue;
           /* s_transmit[i-1].print((unsigned char)(bots[i].id));
            s_transmit[i-1].print((unsigned char)(0));
            s_transmit[i-1].print((unsigned char)(0));*/
            s_transmit.print((unsigned char)(bots[i].id));
            s_transmit.print((unsigned char)(0));
            s_transmit.print((unsigned char)(0));
            //break;
        }
      }
      break;//until escape is pressed
    }
  }
  return 0;
}
