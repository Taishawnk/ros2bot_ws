#include<rclcpp/rclcpp.hpp>
#include<string>
#include<vector>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
#include<memory>


using std::placeholders::_1;//placeholders using it to indicate ny parmChange Callback is expecting one parameter as a Input

class SimpleParameter : public rclcpp::Node
{
    public: //public members
        SimpleParameter() : Node("simple_paramiter")
        {   //used to initialize and declare the parameters on the nodes start up
            declare_parameter<int>("simple_int_param", 28);//18 if no one inter a intager on setup 
            declare_parameter<std::string>("simple_string_param", "Taishawn");//samthing as above but a string

            //change and configure paramiters at runtime
        param_callback_handle_ =  add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));//callback is executed when one or more parameters that are deffined are changed
            //this function is a pointer that indicates we want to use the current instace of the parameters
        }

    private: //members
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; //hoisted to add_on_set_parameters_callback to add the output of it assigned to param_callback_handle
        rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)  //define the parameterChangeCallback function did this after assigning it to teh simple paramter
        {
            rcl_interfaces::msg::SetParametersResult result;
            for(const auto &param : parameters){
                if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER){
                    //sucessfully changed with a new Intager type
                    RCLCPP_INFO_STREAM(get_logger(), "simple_int_param Successfully Changed new value is :" << param.as_int() );
                    result.successful = true;}
                
                // else{
                //     if(param.get_name() != "simple_int_param" && param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER){
                //     //unsucessfully changed with a new Intager type
                //     RCLCPP_INFO_STREAM(get_logger(), "simple_int_param Not Successfully Changed current value is :" << param.as_int() );
                //     result.successful = false;}
                // }

                if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
                    //sucessfully changed with a new String type
                    RCLCPP_INFO_STREAM(get_logger(), "simple_string_param Changed" << param.as_string());
                    result.successful = true;}

            }
        
            return result;
        };//delare the return type of this function so the parameterChangeCallback must return a response of weather there was a successful chage or not
        // this response mus adhere to two standard interfaces that are declared in the rclcpp interface library
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);//intilize the main
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node); //spin the node until it is stopped
    rclcpp::shutdown();
    return 0;
}