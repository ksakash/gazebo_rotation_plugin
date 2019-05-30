#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class ModelRotate : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        this->base_link = this->model->GetLink("base_link");
        if (this->base_link == NULL)
        {
            std::vector<physics::LinkPtr> links = this->model->GetLinks();
            for (int i = 0; i < links.size(); i++)
                std::cout << "Name of link no. " << i << ": " << links[0]->GetName() << std::endl;
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelRotate::OnUpdate, this));
    }

    // Called by the world update start event
public:
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.1));
    }

    // Pointer to the model
private:
    physics::ModelPtr model;

private:
    physics::LinkPtr base_link;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelRotate)
} // namespace gazebo
