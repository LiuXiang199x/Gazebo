/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

class ModelPush : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0));
        math::Pose pose = model->GetWorldPose();
        printf("At: %f %f %f\n", pose.pos.x, pose.pos.y, pose.pos.z);
    }

private:
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
