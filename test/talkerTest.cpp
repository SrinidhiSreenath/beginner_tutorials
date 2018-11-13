/************************************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2018, Srinidhi Sreenath
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    talkerTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    11/13/2018
 *  @version 1.0
 *
 *  @brief Rostest to test the talker node.
 *
 *  @section DESCRIPTION
 *
 *  A rostest using the google test suite to unit test the talker node. The
 *  talker node acts as a publisher node, a server node for a ros service and
 *  also broadcasts a tf frame. The test tests the succesful execution of the
 *  ros service.
 *
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/modifyOutput.h"
#include "std_msgs/String.h"

/**
 * @brief Test case to check the existence of the modifyOutput service

 * @param none
 * @return none
 */
TEST(TestTalkerNode, testInitializationOfROSService) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::modifyOutput>("modify_output");
  bool exists(client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Test case to check the succesful call of the modifyOutput service

 * @param none
 * @return none
 */
TEST(TestTalkerNode, testROSServiceCall) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::modifyOutput>("modify_output");

  beginner_tutorials::modifyOutput::Request req;
  beginner_tutorials::modifyOutput::Response resp;

  req.desiredOutput = "I am BATMAN!";
  std::string precedingString = "The talker node is now publishing: ";
  std::string expectedString = precedingString + req.desiredOutput;

  bool success = client.call(req, resp);
  EXPECT_TRUE(success);
  EXPECT_EQ(expectedString, resp.modifiedOutput);
}
