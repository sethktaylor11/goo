#include "GooHook.h"

using namespace Eigen;

void GooHook::drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu)
{
    if (ImGui::CollapsingHeader("UI Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Combo("Click Adds", (int *)&params_.clickMode, "Particles\0Saws\0\0");
    }
    if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Timestep",  &params_.timeStep);
        ImGui::Combo("Integrator", (int *)&params_.integrator, "Explicit Euler\0Implicit Euler\0Implicit Midpoint\0Velocity Verlet\0\0");
        ImGui::InputDouble("Newton Tolerance", &params_.NewtonTolerance);
        ImGui::InputInt("Newton Max Iters", &params_.NewtonMaxIters);
    }
    if (ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Gravity Enabled", &params_.gravityEnabled);
        ImGui::InputDouble("  Gravity g", &params_.gravityG);
        ImGui::Checkbox("Springs Enabled", &params_.springsEnabled);
        ImGui::InputDouble("  Max Strain", &params_.maxSpringStrain);
        ImGui::Checkbox("Damping Enabled", &params_.dampingEnabled);
        ImGui::InputDouble("  Viscosity", &params_.dampingStiffness);
        ImGui::Checkbox("Floor Enabled", &params_.floorEnabled);
        //viewer.imgui->addWindow(Eigen::Vector2i(1000, 0), "New Objects");
    }


    if (ImGui::CollapsingHeader("New Particles", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Is Fixed", &params_.particleFixed);
        ImGui::InputDouble("Mass", &params_.particleMass);
    }

    if (ImGui::CollapsingHeader("New Saws", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Radius", &params_.sawRadius);
    }

    if (ImGui::CollapsingHeader("New Springs", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Max Spring Dist", &params_.maxSpringDist);
        ImGui::InputDouble("Base Stiffness", &params_.springStiffness);
    }

}

void GooHook::updateRenderGeometry()
{
    double baseradius = 0.02;
    double pulsefactor = 0.1;
    double pulsespeed = 50.0;

    int sawteeth = 20;
    double sawdepth = 0.1;
    double sawangspeed = 10.0;

    double baselinewidth = 0.005;

    int numcirclewedges = 20;

    // this is terrible. But, easiest to get up and running

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3d> vertexColors;
    std::vector<Eigen::Vector3i> faces;

    int idx = 0;

    double eps = 1e-4;


    if(params_.floorEnabled)
    {
        for (int i = 0; i < 6; i++)
        {
            vertexColors.push_back(Eigen::Vector3d(0.3, 1.0, 0.3));
        }

        verts.push_back(Eigen::Vector3d(-1, -0.5, eps));
        verts.push_back(Eigen::Vector3d(1, -0.5, eps));
        verts.push_back(Eigen::Vector3d(-1, -1, eps));

        faces.push_back(Eigen::Vector3i(idx, idx + 1, idx + 2));

        verts.push_back(Eigen::Vector3d(-1, -1, eps));
        verts.push_back(Eigen::Vector3d(1, -0.5, eps));
        verts.push_back(Eigen::Vector3d(1, -1, eps));
        faces.push_back(Eigen::Vector3i(idx + 3, idx + 4, idx + 5));
        idx += 6;
    }


    for (std::vector<Connector *>::iterator it = connectors_.begin(); it != connectors_.end(); ++it)
    {
        Eigen::Vector3d color;
        if ((*it)->associatedBendingStencils.empty())
            color << 0.0, 0.0, 1.0;
        else
            color << 0.75, 0.5, 0.75;
        Vector2d sourcepos = particles_[(*it)->p1].pos;
        Vector2d destpos = particles_[(*it)->p2].pos;

        Vector2d vec = destpos - sourcepos;
        Vector2d perp(-vec[1], vec[0]);
        perp /= perp.norm();

        double dist = (sourcepos - destpos).norm();

        double width = baselinewidth / (1.0 + 20.0 * dist * dist);

        for (int i = 0; i < 4; i++)
            vertexColors.push_back(color);

        verts.push_back(Eigen::Vector3d(sourcepos[0] + width * perp[0], sourcepos[1] + width * perp[1], -eps));
        verts.push_back(Eigen::Vector3d(sourcepos[0] - width * perp[0], sourcepos[1] - width * perp[1], -eps));
        verts.push_back(Eigen::Vector3d(destpos[0] + width * perp[0], destpos[1] + width * perp[1], -eps));
        verts.push_back(Eigen::Vector3d(destpos[0] - width * perp[0], destpos[1] - width * perp[1], -eps));

        faces.push_back(Eigen::Vector3i(idx, idx + 1, idx + 2));
        faces.push_back(Eigen::Vector3i(idx + 2, idx + 1, idx + 3));
        idx += 4;
    }

    int nparticles = particles_.size();

    for(int i=0; i<nparticles; i++)
    {
        double radius = baseradius*sqrt(particles_[i].mass);
        radius *= (1.0 + pulsefactor*sin(pulsespeed*time_));

        Eigen::Vector3d color(0,0,0);

        if(particles_[i].fixed)
        {
            radius = baseradius;
            color << 1.0, 0, 0;
        }

        for (int j = 0; j < numcirclewedges + 2; j++)
        {
            vertexColors.push_back(color);
        }


        verts.push_back(Eigen::Vector3d(particles_[i].pos[0], particles_[i].pos[1], 0));

        const double PI = 3.1415926535898;
        for (int j = 0; j <= numcirclewedges; j++)
        {
            verts.push_back(Eigen::Vector3d(particles_[i].pos[0] + radius * cos(2 * PI*j / numcirclewedges),
                particles_[i].pos[1] + radius * sin(2 * PI*j / numcirclewedges), 0));
        }

        for (int j = 0; j <= numcirclewedges; j++)
        {
            faces.push_back(Eigen::Vector3i(idx, idx + j + 1, idx + 1 + ((j + 1) % (numcirclewedges + 1))));
        }

        idx += numcirclewedges + 2;
    }

    for(std::vector<Saw>::iterator it = saws_.begin(); it != saws_.end(); ++it)
    {
        double outerradius = it->radius;
        double innerradius = (1.0-sawdepth)*outerradius;

        Eigen::Vector3d color(0.5,0.5,0.5);

        int spokes = 2*sawteeth;
        for (int j = 0; j < spokes + 2; j++)
        {
            vertexColors.push_back(color);
        }

        verts.push_back(Eigen::Vector3d(it->pos[0], it->pos[1], 0));

        const double PI = 3.1415926535898;
        for (int i = 0; i <= spokes; i++)
        {
            double radius = (i % 2 == 0) ? innerradius : outerradius;
            verts.push_back(Eigen::Vector3d(it->pos[0] + radius * cos(2 * PI*i / spokes + sawangspeed*time_),
                it->pos[1] + radius * sin(2 * PI*i / spokes + sawangspeed*time_), 0));
        }

        for (int j = 0; j <= spokes; j++)
        {
            faces.push_back(Eigen::Vector3i(idx, idx + j + 1, idx + 1 + ((j + 1) % (spokes + 1))));
        }

        idx += spokes + 2;
    }

    renderQ.resize(verts.size(),3);
    renderC.resize(vertexColors.size(), 3);
    for (int i = 0; i < verts.size(); i++)
    {
        renderQ.row(i) = verts[i];
        renderC.row(i) = vertexColors[i];
    }
    renderF.resize(faces.size(), 3);
    for (int i = 0; i < faces.size(); i++)
        renderF.row(i) = faces[i];
}


void GooHook::initSimulation()
{
    time_ = 0;
    particles_.clear();
    for(std::vector<Connector *>::iterator it = connectors_.begin(); it != connectors_.end(); ++it)
        delete *it;
    connectors_.clear();
    saws_.clear();
}

void GooHook::tick()
{
    message_mutex.lock();
    {
        while (!mouseClicks_.empty())
        {
            MouseClick mc = mouseClicks_.front();
            mouseClicks_.pop_front();
            switch (mc.mode)
            {
            case SimParameters::ClickMode::CM_ADDPARTICLE:
            {
                addParticle(mc.x, mc.y);
                break;
            }
            case SimParameters::ClickMode::CM_ADDSAW:
            {
                addSaw(mc.x, mc.y);
                break;
            }
            }
        }
    }
    message_mutex.unlock();
}

bool GooHook::simulateOneStep()
{
    // TODO: implement time integration
    VectorXd q;
    VectorXd q_0;
    VectorXd q_dot;
    buildConfig(q, q_0, q_dot);

    SparseMatrix<double> M;
    SparseMatrix<double> M_inv;
    computeMassMatrices(M, M_inv);

    VectorXd q_1 = q + params_.timeStep * q_dot;
    VectorXd q_dot_1;

    RowVectorXd F;

    switch (params_.integrator)
    {
        case params_.TI_EXPLICIT_EULER:
            computeForces(q, q_0, M, F);

            q_dot_1 = q_dot + params_.timeStep * M_inv * F.transpose();

            break;
	case params_.TI_VELOCITY_VERLET:
	    computeForces(q_1, q, M, F);

            q_dot_1 = q_dot + params_.timeStep * M_inv * F.transpose();

            break;
        case params_.TI_IMPLICIT_EULER:
            for (int i = 0; i < params_.NewtonMaxIters; i++) {
                VectorXd f = residualImplicit(q_1,q,q_dot,M,M_inv);
                if (f.norm() < params_.NewtonTolerance)
                    break;
                SparseMatrix<double> I(particles_.size()*2,particles_.size()*2);
                I.setIdentity();
		SparseMatrix<double> dF;
		computedF(q_1,dF);
		SparseMatrix<double> df = I - pow(params_.timeStep,2) * M_inv * dF;
                SimplicialLLT<SparseMatrix<double>> solver;
                solver.compute(df);
                VectorXd dq = solver.solve(-f);
		q_1 += dq;
	    }

	    computeForces(q_1, q, M, F);

            q_dot_1 = q_dot + params_.timeStep * M_inv * F.transpose();

            break;
	case params_.TI_IMPLICIT_MIDPOINT:
            for (int i = 0; i < params_.NewtonMaxIters; i++) {
                VectorXd f = residualMidpoint(q_1,q,q_0,q_dot,M,M_inv);
                if (f.norm() < params_.NewtonTolerance)
                    break;
                SparseMatrix<double> I(particles_.size()*2,particles_.size()*2);
                I.setIdentity();
		SparseMatrix<double> dF;
		computedF((q_1+q)/2,dF);
		SparseMatrix<double> df = I - pow(params_.timeStep,2) * M_inv * dF;
                SimplicialLLT<SparseMatrix<double>> solver;
                solver.compute(df);
                VectorXd dq = solver.solve(-f);
		q_1 += dq;
	    }

	    computeForces((q_1+q)/2, (q+q_0)/2, M, F);

            q_dot_1 = q_dot + params_.timeStep * M_inv * F.transpose();

            break;
    }

    storeConfig(q_1, q_dot_1);

    time_ += params_.timeStep;
    return false;
}

void GooHook::addParticle(double x, double y)
{
    Vector2d newpos(x,y);
    double mass = params_.particleMass;
    if(params_.particleFixed)
        mass = std::numeric_limits<double>::infinity();

    int newid = particles_.size();
    particles_.push_back(Particle(newpos, mass, params_.particleFixed, false));

    // TODO connect particles with springs
    for (int i = 0; i < particles_.size()-1; i++) {
        Particle p = particles_[i];
	double restlen = (newpos - p.pos).norm();
        if (restlen <= params_.maxSpringDist) {
            connectors_.push_back(new Spring(i,particles_.size()-1,0,params_.springStiffness,restlen,true));
	}
    }
}

void GooHook::addSaw(double x, double y)
{
    saws_.push_back(Saw(Vector2d(x,y), params_.sawRadius));
}

void GooHook::buildConfig(VectorXd &q, VectorXd &q_0, VectorXd &q_dot)
{
    q.resize(particles_.size()*2);
    q_0.resize(particles_.size()*2);
    q_dot.resize(particles_.size()*2);
    for (int i = 0; i < particles_.size(); i++) {
        q[2*i] = particles_[i].pos[0];
        q[2*i+1] = particles_[i].pos[1];
        q_0[2*i] = particles_[i].prevpos[0];
        q_0[2*i+1] = particles_[i].prevpos[1];
        q_dot[2*i] = particles_[i].vel[0];
        q_dot[2*i+1] = particles_[i].vel[1];
    }
}

void GooHook::storeConfig(VectorXd q, VectorXd q_dot)
{
    for (int i = 0; i < particles_.size(); i++) {
        particles_[i].prevpos = particles_[i].pos;
        particles_[i].pos = Vector2d(q[2*i],q[2*i+1]);
        particles_[i].vel = Vector2d(q_dot[2*i],q_dot[2*i+1]);
    }
}

void GooHook::computeForces(VectorXd q, VectorXd q_0, SparseMatrix<double> M, RowVectorXd &F)
{
    F.resize(q.size());
    F.setZero();
    if (params_.gravityEnabled) {
        RowVectorXd S_g(q.size());
        for (int i = 0; i < q.size(); i+=2) {
            S_g[i] = 0;
            S_g[i+1] = 1;
        }
        F += params_.gravityG * S_g * M;
    }
    if (params_.springsEnabled) {
        for (int i = 0; i < connectors_.size(); i++) {
            Spring* s = (Spring *)connectors_[i];

            MatrixXd S(2,q.size());
	    S.setZero();
	    S(0,2*s->p1) = -1;
	    S(1,2*s->p1+1) = -1;
	    S(0,2*s->p2) = 1;
	    S(1,2*s->p2+1) = 1;

	    Vector2d vec = S * q;

            F += -s->stiffness/s->restlen*(vec.norm()-s->restlen)*vec.transpose()/vec.norm()*S;
        }
    }
    if (params_.dampingEnabled) {
        for (int i = 0; i < connectors_.size(); i++) {
            Spring* s = (Spring *)connectors_[i];

            MatrixXd S(2,q.size());
	    S.setZero();
	    S(0,2*s->p1) = -1;
	    S(1,2*s->p1+1) = -1;
	    S(0,2*s->p2) = 1;
	    S(1,2*s->p2+1) = 1;

	    Vector2d vec = - (S * (q - q_0)) / params_.timeStep;

            F += params_.dampingStiffness * vec.transpose() * S;
        }
    }
    for (int i = 0; i < particles_.size(); i++) {
        if (particles_[i].fixed) {
             F[2*i] = 0;
	     F[2*i+1] = 0;
	}
    }
}

void GooHook::computedF(VectorXd q, SparseMatrix<double> &dF)
{
    dF.resize(q.size(),q.size());
    if (params_.springsEnabled) {
        SparseMatrix<double> dummy(q.size(),q.size());
        std::vector<Triplet<double>> dF_entries;
        dF_entries.resize(connectors_.size()*2);
        for (int i = 0; i < connectors_.size(); i++) {
            Spring* s = (Spring *)connectors_[i];

            dF_entries[2*i] = Triplet<double>(s->p1,s->p2,-s->stiffness);
            dF_entries[2*i+1] = Triplet<double>(s->p2,s->p1,-s->stiffness);
        }
        dummy.setFromTriplets(dF_entries.begin(),dF_entries.end());
	dF += dummy;
    }
    if (params_.dampingEnabled) {
        SparseMatrix<double> dummy(q.size(),q.size());
        std::vector<Triplet<double>> dF_entries;
        dF_entries.resize(connectors_.size()*2);
        for (int i = 0; i < connectors_.size(); i++) {
            Spring* s = (Spring *)connectors_[i];

            dF_entries[2*i] = Triplet<double>(s->p1,s->p2,-params_.dampingStiffness/params_.timeStep);
            dF_entries[2*i+1] = Triplet<double>(s->p2,s->p1,-params_.dampingStiffness/params_.timeStep);
        }
        dummy.setFromTriplets(dF_entries.begin(),dF_entries.end());
	dF += dummy;
    }
}

void GooHook::computeMassMatrices(SparseMatrix<double> &M, SparseMatrix<double> &M_inv)
{
    M.resize(particles_.size()*2,particles_.size()*2);
    std::vector<Triplet<double>> mass_entries;
    mass_entries.resize(particles_.size()*2);
    for (int i = 0; i < particles_.size(); i++) {
        mass_entries[2*i] = Triplet<double>(2*i,2*i,particles_[i].mass);
        mass_entries[2*i+1] = Triplet<double>(2*i+1,2*i+1,particles_[i].mass);
    }
    M.setFromTriplets(mass_entries.begin(),mass_entries.end());
    SimplicialLLT<SparseMatrix<double>> solver;
    solver.compute(M);
    SparseMatrix<double> I(particles_.size()*2,particles_.size()*2);
    I.setIdentity();
    M_inv = solver.solve(I);
}

VectorXd GooHook::residualImplicit(VectorXd q_1, VectorXd q, VectorXd q_dot, SparseMatrix<double> M, SparseMatrix<double> M_inv)
{
    RowVectorXd F;
    computeForces(q_1, q, M, F);
    return q_1 - q - params_.timeStep * q_dot - pow(params_.timeStep,2) * M_inv * F.transpose();
}

VectorXd GooHook::residualMidpoint(VectorXd q_1, VectorXd q, VectorXd q_0, VectorXd q_dot, SparseMatrix<double> M, SparseMatrix<double> M_inv)
{
    RowVectorXd F;
    computeForces((q_1+q)/2, (q+q_0)/2, M, F);
    return q_1 - q - params_.timeStep * q_dot - pow(params_.timeStep,2) / 2 * M_inv * F.transpose();
}
