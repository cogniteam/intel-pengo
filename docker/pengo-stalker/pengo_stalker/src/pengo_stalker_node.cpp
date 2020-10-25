/**
 * @brief 
 * 
 * @file pengo_stalker_node.cpp
 * 
 * @author Igor Makhtes (igor@cogniteam.com)
 * @date 2020-10-13
 * @copyright Cogniteam (c) 2020
 * 
 * Cogniteam LTD
 *   
 * Unpublished Copyright (c) 2016-2020 Cogniteam
 *    
 * 
 */


#include <pengo_stalker/PengoStalkerRos.h>


/**
 * @brief 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "pengo_stalker_node");
    ros::NodeHandle nodeHandle;
    pengo::PengoStalkerRos pengoStalkerRos;    
    return 0;
}
