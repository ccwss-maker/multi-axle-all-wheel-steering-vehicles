#include <car.hpp>

using namespace ros;
using namespace std;
using namespace sensor_msgs;

string config_yaml_path = "/home/ccwss/PersonalData/Program/Ros/car5_ws/src/robot/robot1_program/config/size.yaml";
string output_data_path = "/home/ccwss/PersonalData/Program/Ros/car5_ws/output_data.txt";

struct timeval tv;
car_ctrl_ car_ctrl;

car_info_ car_info;
img_data_ center_line_of_road;
img_data_ center_of_steering;
ofstream txt_out;

ros::Publisher left_wheel_pub[4];
ros::Publisher right_wheel_pub[4];
ros::Publisher left_string_pub[4];
ros::Publisher right_string_pub[4];
ros::Publisher a_deviation_pub;
ros::Publisher H_deviation_pub;
ros::Publisher dl_deviation_pub;
ros::Publisher dr_deviation_pub;

std_msgs::Float64 H_deviation;
std_msgs::Float64 a_deviation;
std_msgs::Float64 dl_deviation;
std_msgs::Float64 dr_deviation;

class SubscribeAndPublish
{
    public:
    SubscribeAndPublish()
    {
        //publish
        left_string_pub[0]= n_.advertise<std_msgs::Float64>("/car/left_string_1_position_controller/command", 1);
        left_string_pub[1]= n_.advertise<std_msgs::Float64>("/car/left_string_2_position_controller/command", 1);
        left_string_pub[2]= n_.advertise<std_msgs::Float64>("/car/left_string_3_position_controller/command", 1);
        left_string_pub[3]= n_.advertise<std_msgs::Float64>("/car/left_string_4_position_controller/command", 1);
        right_string_pub[0]= n_.advertise<std_msgs::Float64>("/car/right_string_1_position_controller/command", 1);
        right_string_pub[1]= n_.advertise<std_msgs::Float64>("/car/right_string_2_position_controller/command", 1);
        right_string_pub[2]= n_.advertise<std_msgs::Float64>("/car/right_string_3_position_controller/command", 1);
        right_string_pub[3]= n_.advertise<std_msgs::Float64>("/car/right_string_4_position_controller/command", 1);

        left_wheel_pub[0]= n_.advertise<std_msgs::Float64>("/car/left_wheel_1_velocity_controller/command", 1);
        left_wheel_pub[1]= n_.advertise<std_msgs::Float64>("/car/left_wheel_2_velocity_controller/command", 1);
        left_wheel_pub[2]= n_.advertise<std_msgs::Float64>("/car/left_wheel_3_velocity_controller/command", 1);
        left_wheel_pub[3]= n_.advertise<std_msgs::Float64>("/car/left_wheel_4_velocity_controller/command", 1);
        right_wheel_pub[0]= n_.advertise<std_msgs::Float64>("/car/right_wheel_1_velocity_controller/command", 1);
        right_wheel_pub[1]= n_.advertise<std_msgs::Float64>("/car/right_wheel_2_velocity_controller/command", 1);
        right_wheel_pub[2]= n_.advertise<std_msgs::Float64>("/car/right_wheel_3_velocity_controller/command", 1);
        right_wheel_pub[3]= n_.advertise<std_msgs::Float64>("/car/right_wheel_4_velocity_controller/command", 1);
        
        H_deviation_pub= n_.advertise<std_msgs::Float64>("/car/deviation/H", 1);
        a_deviation_pub= n_.advertise<std_msgs::Float64>("/car/deviation/a", 1);
        dl_deviation_pub= n_.advertise<std_msgs::Float64>("/car/deviation/dl", 1);
        dr_deviation_pub= n_.advertise<std_msgs::Float64>("/car/deviation/dr", 1);
        //subscribe
        odom_sub = n_.subscribe("/gazebo/model_states", 1, &SubscribeAndPublish::odom_callback, this);
        laser_sub_1.subscribe(n_, "/car/laser_1/scan", 1);
        laser_sub_2.subscribe(n_, "/car/laser_2/scan", 1);
        laser_sub_3.subscribe(n_, "/car/laser_3/scan", 1);
        laser_sub_4.subscribe(n_, "/car/laser_4/scan", 1);

        sync.reset(new Sync(MySyncPolicy(1), laser_sub_1,laser_sub_2,laser_sub_3,laser_sub_4));
        sync->registerCallback(boost::bind(&laser_callback, _1, _2,_3,_4));
    }
    void odom_callback(const gazebo_msgs::ModelStatesConstPtr & msg);
    
    private:
    NodeHandle n_;
    Subscriber joint_states_sub;
    ros::Subscriber kbd_sub;
    ros::Subscriber odom_sub;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_1;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_2;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_3;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_4;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::LaserScan,sensor_msgs::LaserScan,sensor_msgs::LaserScan,sensor_msgs::LaserScan> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    std_msgs::Float64 pub_left_string;
    std_msgs::Float64 pub_right_string;
};

void SubscribeAndPublish::odom_callback(const gazebo_msgs::ModelStatesConstPtr & msg)
{
    cv::Point2d p;
    p.x = msg->pose[2].position.x;
    p.y = msg->pose[2].position.y;
    if(car_info.car_position.size() == 0 || fabs(p.x - car_info.car_position.back().x) > 0.01 || fabs(p.y - car_info.car_position.back().y) > 0.01)
        car_info.car_position.push_back(p);

    if(car_info.car_position.size() >= 3000)
        car_info.car_position.erase(car_info.car_position.begin());

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void Img_Mouse_Callback(int event, int x, int y, int flags, void* param)
{
    static int x_last;
    static int y_last;
    img_data_* img_data = (img_data_*)param;
    switch(event)
    {
        case cv::EVENT_MOUSEHWHEEL:
        {
            int value = cv::getMouseWheelDelta(flags);
            if(value < 0)   
                img_data->Mat_scaling+=0.1;
            else if(value > 0 && img_data->Mat_scaling > 0.1)
                img_data->Mat_scaling-=0.1;

            img_data->u0_x += (img_data->Mat.cols / 2 - x) / 5.0;
            img_data->u0_y += (img_data->Mat.rows / 2 - y) / 5.0;
            break;
        }
        case cv::EVENT_MBUTTONDOWN:
        {
            img_data->Mat_scaling = 1;
            img_data->u0_x = img_data->Mat.cols / 2;
            img_data->u0_y = img_data->Mat.rows / 2;
            break;
        }
        case cv::EVENT_LBUTTONDOWN:
        {
            cv::imwrite("/home/ccwss/PersonalData/Program/Ros/car5_ws/0.jpg", img_data->Mat);
            // img_data->u0_x += (img_data->Mat.cols / 2 - x);
            // img_data->u0_y += (img_data->Mat.rows / 2 - y);
            break;
        }
        case cv::EVENT_RBUTTONDOWN:
        {
            x_last = x;
            y_last = y;
            break;
        }
        case cv::EVENT_MOUSEMOVE:
        {
            if(flags == cv::EVENT_FLAG_RBUTTON)
            {
                int x_diff = x-x_last;
                int y_diff = y-y_last;
                img_data->u0_x += x_diff;
                img_data->u0_y += y_diff;
                x_last = x;
                y_last = y;
            }
        }
    }
}

void body_init()
{
    car_info.body_car[0]=cv::Point2d(car_info.length/2,car_info.width/2);
    car_info.body_car[1]=cv::Point2d(-car_info.length/2,car_info.width/2);
    car_info.body_car[2]=cv::Point2d(-car_info.length/2,-car_info.width/2);
    car_info.body_car[3]=cv::Point2d(car_info.length/2,-car_info.width/2);
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<4;j++)
        {
            car_info.body_wheel[i][j].point= cv::Point2d(car_info.wheel_x[j], car_info.wheel_y[i]);
        }
    }
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& laser1, const sensor_msgs::LaserScanConstPtr & laser2,const sensor_msgs::LaserScanConstPtr & laser3 ,const sensor_msgs::LaserScanConstPtr & laser4)
{ 
    gettimeofday(&tv,NULL);
    static int init_sign=0;
    if(init_sign==0)
    {
        body_init();
        cv::namedWindow("center_of_steering", cv::WINDOW_NORMAL);
        cv::resizeWindow("center_of_steering", 500, 500);
        cv::setMouseCallback("center_of_steering", Img_Mouse_Callback, &center_of_steering);
        center_of_steering.Mat = cv::Mat::zeros(500,500,CV_8UC3);
        center_of_steering.Mat_scaling = 1;
        center_of_steering.obj_width = 5;
        center_of_steering.u0_x = center_of_steering.Mat.cols / 2;
        center_of_steering.u0_y = center_of_steering.Mat.rows / 2;
        
        cv::namedWindow("center_line_of_road", cv::WINDOW_NORMAL);
        cv::resizeWindow("center_line_of_road", 500, 500);
        cv::setMouseCallback("center_line_of_road", Img_Mouse_Callback, &center_line_of_road);
        center_line_of_road.Mat = cv::Mat::zeros(500,500,CV_8UC3);
        center_line_of_road.Mat_scaling = 1;
        center_line_of_road.obj_width = 320;
        center_line_of_road.u0_x = center_line_of_road.Mat.cols / 2;
        center_line_of_road.u0_y = center_line_of_road.Mat.rows / 2;

        init_sign=1;
    }

    float d1 = laser4->ranges[0]-0.01;
    float d4 = laser1->ranges[0]-0.01;
    float d2 = laser3->ranges[0]-0.01;
    float d3 = laser2->ranges[0]-0.01;
    car_ctrl_ auto_ctrl = cat_auto_ctrl(d1,d4,d2,d3);

    /*画车身*/
    center_of_steering.Mat = cv::Scalar(255,255,255);
    center_of_steering.u_x = (double)center_of_steering.Mat.cols / center_of_steering.obj_width * center_of_steering.Mat_scaling;
    center_of_steering.u_y = (double)center_of_steering.Mat.rows / center_of_steering.obj_width * center_of_steering.Mat_scaling;
    car_info.body_car_uv[0]=Touv(car_info.body_car[0] , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
    car_info.body_car_uv[1]=Touv(car_info.body_car[1] , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
    car_info.body_car_uv[2]=Touv(car_info.body_car[2] , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
    car_info.body_car_uv[3]=Touv(car_info.body_car[3] , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
    cv::line(center_of_steering.Mat,car_info.body_car_uv[0],car_info.body_car_uv[1],cv::Scalar(0,255,0),2,cv::LINE_AA);
    cv::line(center_of_steering.Mat,car_info.body_car_uv[1],car_info.body_car_uv[2],cv::Scalar(0,255,0),2,cv::LINE_AA);
    cv::line(center_of_steering.Mat,car_info.body_car_uv[2],car_info.body_car_uv[3],cv::Scalar(0,255,0),2,cv::LINE_AA);
    cv::line(center_of_steering.Mat,car_info.body_car_uv[3],car_info.body_car_uv[0],cv::Scalar(0,255,0),2,cv::LINE_AA);
    /*画转向中心*/
    for(int i=0 ; i<2 ; i++)
    {
        for(int j=0 ; j<4 ; j++)
        {
            double k;
            if(i == 0)  k= -1.0 /tan(auto_ctrl.pub_left_string[j].data);
            else        k= -1.0 /tan(auto_ctrl.pub_right_string[j].data);
            double x0=car_info.body_wheel[i][j].point.x;
            double y0=car_info.body_wheel[i][j].point.y;
            cout<<i<<','<<j<<','<<k<<','<<y0-k*x0<<endl;
            car_info.body_wheel[i][j].point_boundary[0].x = -(double)center_of_steering.obj_width / 2 / center_of_steering.Mat_scaling;
            car_info.body_wheel[i][j].point_boundary[1].x = (double)center_of_steering.obj_width / 2 / center_of_steering.Mat_scaling;
            car_info.body_wheel[i][j].point_boundary[0].y = k*(car_info.body_wheel[i][j].point_boundary[0].x-x0)+y0;
            car_info.body_wheel[i][j].point_boundary[1].y = k*(car_info.body_wheel[i][j].point_boundary[1].x-x0)+y0;

            cv::Scalar color;
            if(i == 0)  color = cv::Scalar(255,0,0);
            else        color = cv::Scalar(0,0,255);
            car_info.body_wheel_uv[i][j].point = Touv(car_info.body_wheel[i][j].point , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
            car_info.body_wheel_uv[i][j].point_boundary[0] = Touv(car_info.body_wheel[i][j].point_boundary[0], center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
            car_info.body_wheel_uv[i][j].point_boundary[1] = Touv(car_info.body_wheel[i][j].point_boundary[1] , center_of_steering.u_x , center_of_steering.u_y , center_of_steering.u0_x , center_of_steering.u0_y);
            cv::line(center_of_steering.Mat,car_info.body_wheel_uv[i][j].point_boundary[0],car_info.body_wheel_uv[i][j].point_boundary[1],color,1,cv::LINE_AA);
            cv::circle(center_of_steering.Mat,car_info.body_wheel_uv[i][j].point,5,cv::Scalar(0,0,0),-1);
        }
    }
    cout<<endl;
    /*画道路中心线*/
    center_line_of_road.Mat = cv::Scalar(255,255,255);
    center_line_of_road.u_x = (double)center_line_of_road.Mat.cols / center_line_of_road.obj_width * center_line_of_road.Mat_scaling;
    center_line_of_road.u_y = (double)center_line_of_road.Mat.rows / center_line_of_road.obj_width * center_line_of_road.Mat_scaling;
    cv::Point p1 = Touv(cv::Point2d(0, 50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::Point p2 = Touv(cv::Point2d(0, -50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::Point p3 = Touv(cv::Point2d(-101.25, -50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::Point p4 = Touv(cv::Point2d(-101.25, 50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::Point p5 = Touv(cv::Point2d(101.25, -50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::Point p6 = Touv(cv::Point2d(101.25, 50), center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
    cv::ellipse(center_line_of_road.Mat, p1, cv::Size(101.25*center_line_of_road.u_x, 101.25*center_line_of_road.u_y), 180, 0, 180, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    cv::ellipse(center_line_of_road.Mat, p2, cv::Size(101.25*center_line_of_road.u_x, 101.25*center_line_of_road.u_y), 0, 0, 180, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    cv::line(center_line_of_road.Mat , p3 , p4 , cv::Scalar(0,0,255),1,cv::LINE_AA );
    cv::line(center_line_of_road.Mat , p5 , p6 , cv::Scalar(0,0,255),1,cv::LINE_AA );
    /*画车身轨迹*/
    for(int i = 0; i<car_info.car_position.size(); i++)
    {
        cv::Point car_position_uv = Touv(car_info.car_position[i], center_line_of_road.u_x, center_line_of_road.u_y, center_line_of_road.u0_x, center_line_of_road.u0_y);
        cv::circle(center_line_of_road.Mat, car_position_uv, 1, cv::Scalar(0,0,0), -1);
    }

    cv::imshow("center_of_steering",center_of_steering.Mat);
    cv::imshow("center_line_of_road",center_line_of_road.Mat);
    cv::waitKey(1);

}

cv::Point2d Touv(cv::Point2d P0 , double u_x , double u_y , int u0 , int v0)
{
    cv::Point2d P;
    P.x = u_x*P0.x+u0;
    P.y = -u_y*P0.y+v0;

    return P;
}

int main(int argc, char **argv)
{
    car_info.length = 4;
    car_info.width = 2;
    car_info.laser_length = 3.98;
    car_info.laser_width = 1;
    car_info.wheel_x[0] = 1.6;
    car_info.wheel_x[1] = 0.6;
    car_info.wheel_x[2] = -0.6;
    car_info.wheel_x[3] = -1.6;
    car_info.wheel_y[0] = 0.5;
    car_info.wheel_y[1] = -0.5;
    car_info.k_v = 1;
    car_info.k_a = 2;
    car_info.k_h = 1;

    txt_out.open(output_data_path, std::ios::out | std::ios::app);
    //Initiate ROS
    ros::init(argc, argv, "subscribe_and_publish", ros::init_options::NoSigintHandler);

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish test;
    ros::MultiThreadedSpinner s(2);  //多线程
    ros::spin(s);

    return 0;
}

car_ctrl_ cat_auto_ctrl(float d1,float d4,float d2,float d3)
{
    car_ctrl_ car_ctrl_auto;
    double L1;
    double angle_a = atan((double)(d2-d1)/car_info.laser_length);
    if(angle_a>0)   L1 = (d4-d2)/(d2-d1)*car_info.laser_length;
    else            L1 = (d1-d3)/(d3-d4)*car_info.laser_length;
    double L2 = L1/2;
    double H = L2*sin(angle_a);
    car_info.L2 = L2;
    car_info.H = H;
    cout<<angle_a<<','<<car_info.H<<endl;
    YAML::Node point_node = YAML::LoadFile(config_yaml_path);
    car_info.k_a = point_node["k_a"].as<std::double_t>();
    car_info.k_h = point_node["k_h"].as<std::double_t>();
    car_info.k_v = point_node["k_v"].as<std::double_t>();

    for(int i = 0 ; i < 4 ; i++)
    {
        double X_wi = car_info.wheel_x[i];
        double angle_b = atan(car_info.laser_width/(2*(X_wi+L2)));

        double H_LR = -car_info.k_a*angle_a*car_info.laser_width/(2*tan(angle_b)) - car_info.k_v*sin(angle_a) - car_info.k_h*H*cos(angle_a);
        double V_R = -car_info.k_a*angle_a*car_info.laser_width/2 + car_info.k_v*cos(angle_a) - car_info.k_h*H*sin(angle_a);
        double V_L = car_info.k_a*angle_a*car_info.laser_width/2 + car_info.k_v*cos(angle_a) - car_info.k_h*H*sin(angle_a);

        car_ctrl_auto.pub_right_string[i].data = atan(H_LR / V_R);
        car_ctrl_auto.pub_left_string[i].data = atan(H_LR / V_L);
        
        car_ctrl_auto.pub_right_wheel[i].data = car_info.k_v/fabs(car_info.k_v)*pow(pow(V_R,2)+pow(H_LR,2),0.5);
        car_ctrl_auto.pub_left_wheel[i].data = car_info.k_v/fabs(car_info.k_v)*pow(pow(V_L,2)+pow(H_LR,2),0.5);
    }

    for(int i=0;i<4;i++)
    {
        left_wheel_pub[i].publish(car_ctrl_auto.pub_left_wheel[i]);
        right_wheel_pub[i].publish(car_ctrl_auto.pub_right_wheel[i]);
        left_string_pub[i].publish(car_ctrl_auto.pub_left_string[i]);
        right_string_pub[i].publish(car_ctrl_auto.pub_right_string[i]);
    }

    H_deviation.data = L2*sin(angle_a);
    a_deviation.data = angle_a*180/PI;
    dl_deviation.data = d1;
    dr_deviation.data = d4;
    txt_out<<tv.tv_sec*1000+tv.tv_usec/1000<<','<<a_deviation.data<<','<<H_deviation.data<<','<<dl_deviation.data<<','<<dr_deviation.data<<"\n";

    H_deviation_pub.publish(H_deviation);
    a_deviation_pub.publish(a_deviation);
    dl_deviation_pub.publish(dl_deviation);
    dr_deviation_pub.publish(dr_deviation);

    return car_ctrl_auto;

}
