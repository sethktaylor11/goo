#include "PhysicsHook.h"
#include "SceneObjects.h"
#include <deque>
#include "SimParameters.h"
#include <Eigen/Sparse>
#include <Eigen/StdVector>

struct MouseClick
{
    double x;
    double y;
    SimParameters::ClickMode mode;
};

class GooHook : public PhysicsHook
{
public:
    GooHook() : PhysicsHook() {}

    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu);

    virtual void initSimulation();

    virtual void mouseClicked(double x, double y, int button)
    {
        message_mutex.lock();
        {
            MouseClick mc;
            mc.x = x;
            mc.y = y;
            mc.mode = params_.clickMode;
            mouseClicks_.push_back(mc);
        }
        message_mutex.unlock();
    }

    virtual void updateRenderGeometry();

    virtual void tick();

    virtual bool simulateOneStep();

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer)
    {
        viewer.data().clear();
        viewer.data().set_mesh(renderQ, renderF);
        viewer.data().set_colors(renderC);
    }

private:
    SimParameters params_;
    double time_;
    std::vector<Particle, Eigen::aligned_allocator<Particle> > particles_;
    std::vector<Connector *> connectors_;
    std::vector<Saw> saws_;

    std::mutex message_mutex;
    std::deque<MouseClick> mouseClicks_;

    Eigen::MatrixXd renderQ;
    Eigen::MatrixXi renderF;
    Eigen::MatrixXd renderC;

    void addParticle(double x, double y);
    void addSaw(double x, double y);

    void buildConfig(Eigen::VectorXd &q, Eigen::VectorXd &q_0, Eigen::VectorXd &q_dot);
    void storeConfig(Eigen::VectorXd q, Eigen::VectorXd q_dot);

    void computeForces(Eigen::VectorXd q, Eigen::VectorXd q_0, Eigen::SparseMatrix<double> M, Eigen::RowVectorXd &F);
    void computedF(Eigen::VectorXd q, Eigen::SparseMatrix<double> &dF);

    void computeMassMatrices(Eigen::SparseMatrix<double> &M, Eigen::SparseMatrix<double> &M_inv);

    Eigen::VectorXd residualImplicit(Eigen::VectorXd q_1, Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> M_inv);
    Eigen::VectorXd residualMidpoint(Eigen::VectorXd q_1, Eigen::VectorXd q, Eigen::VectorXd q_0, Eigen::VectorXd q_dot, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> M_inv);
};
