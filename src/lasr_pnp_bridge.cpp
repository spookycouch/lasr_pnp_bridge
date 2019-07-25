#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <actionlib/client/simple_action_client.h>

#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>
#include <lasr_pnp_bridge/bridgeAction.h>

typedef actionlib::SimpleActionClient<lasr_pnp_bridge::bridgeAction> ActionClient;

class GenericActionServer : public PNPActionServer
{
    private:
        std::string robotname;
        ros::NodeHandle handle;
        ros::Publisher condition_pub;
        ros::Subscriber condition_sub;
        ros::ServiceClient variable_client;
        std::map<string, ActionClient*> action_clients;

    public:

        // external_action
        //
        // a PNP action, allowing PNP commands to be passed to external action servers
        // (e.g. a simple action server written in Python)
        //
        // actions in PNP are to be of the form:
        // extern_topicName/actionName_param1_param2_param3_...
        //
        // external_action will query a dictionary for a client to the specified topic,
        // and will create and cache one if not found.
        //
        // the topic is a server, where the actionName and params will be passed as a goal.
        //
        // returned results are exit status, condition events and set variable calls,
        // where the latter two are passed to PNP to modify the flow of a plan.
        //
        //
        void external_action(string params, bool *run)
        {
            // extract topic and action_cmd from params
            int split_index = params.find_last_of('/');
            std::string topic = params.substr(0, split_index);
            std::string action_cmd = params.substr(split_index + 1);

            // create new client if it does not already exist
            if(action_clients[topic] == NULL)
            {
                ROS_INFO_STREAM("Creating new client for server at topic=" << topic);
                ActionClient *ac = new ActionClient(topic, true);
                // wait for external server
                while(!ac->waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("Waiting for external server to come up");
                }  
                action_clients[topic] = ac;
            }
            else
            {
                ROS_INFO_STREAM("Client for topic=" << topic << " already exists");
            }

            // create goal
            lasr_pnp_bridge::bridgeGoal goal;
            
            // extract action and params from action_cmd
            std::replace(action_cmd.begin(), action_cmd.end(), '_', ' ');
            std::stringstream ss(action_cmd);
            std::string temp_param;
            ss >> goal.action;
            while(ss >> temp_param)
                goal.params.push_back(temp_param);

            // send goal
            ROS_INFO("Sending goal (action & params) to external server...");
            action_clients[topic]->sendGoal(goal);

            // wait for result
            while(*run && !action_clients[topic]->waitForResult());
            if(!*run)
            {
                ROS_INFO_STREAM("Action " << goal.action << "interrupted!");
                return;
            }

            // get result
            lasr_pnp_bridge::bridgeResultConstPtr result = action_clients[topic]->getResult();
            ROS_INFO_STREAM("Action completed. exit_status=" << result->exit_status);

            // publish PNP condition events
            for(unsigned i = 0; i < result->condition_event.size(); ++i)
            {
                std_msgs::String condition_out;
                condition_out.data = result->condition_event[i];
                condition_pub.publish(condition_out);
            }

            // call PNP set variable service
            for(unsigned i = 0; i < result->set_variable_variable.size(); ++i)
            {
                pnp_msgs::PNPSetVariableValue variable_out;
                variable_out.request.variable = result->set_variable_variable[i];
                variable_out.request.value = result->set_variable_value[i];
                variable_client.call(variable_out);
            }
        }


        // Relay any string message in as a PNPConditionEvent
        //
        //
        void condition_callback(const std_msgs::String::ConstPtr& msg)
        {
            condition_pub.publish(msg);
        }


        // Constructor
        //
        // initialise PNP publishers, subscribers and serviceClients
        // register the external action
        //
        //
        GenericActionServer() : PNPActionServer()
        {
            // initialise publisher and subscriber for condition events
            condition_pub = handle.advertise<std_msgs::String>("PNPConditionEvent", 10);
            condition_sub = handle.subscribe("PNPConditionIn", 10, &GenericActionServer::condition_callback, this);
            variable_client = handle.serviceClient<pnp_msgs::PNPSetVariableValue>("PNPSetVariableValue");

            // set the robot name
            handle.param("robot_name",robotname,std::string(""));
	        ROS_INFO("ROBOTNAME: %s",robotname.c_str());

            // register external action
            register_action("extern", &GenericActionServer::external_action, this);
        }
};


// main
//
//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "genericPNPAS");
    GenericActionServer genericPNPAS;
    genericPNPAS.start();
    ros::spin();
}