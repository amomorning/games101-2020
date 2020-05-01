#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        
        double dx = (end.x - start.x) / (num_nodes - 1);
        double dy = (end.y - start.y) / (num_nodes - 1);
        
        for(int i = 0; i < num_nodes; ++i) {
            auto pos = Vector2D(start.x + dx * i, start.y + dy * i);
            masses.push_back(new Mass(pos, node_mass, false));
            if(i) springs.push_back(new Spring(masses[i-1], masses[i], k));
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // Use Hooke's law to calculate the force on a node
            auto m1 = s->m1;
            auto m2 = s->m2;

            double l = (m1->start_position - m2->start_position).norm();
            double d = (m1->position - m2->position).norm();

            m1->forces += -(s->k) * (m1->position - m2->position) * (d-l)/d;
            m2->forces += -(s->k) * (m2->position - m1->position) * (d-l)/d;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                auto a = (m->forces + gravity) / m->mass;
                auto cur_position = m->position;
                auto cur_velocity = m->velocity;
                double damping_factor = 0.00005;
                std::cout << cur_velocity << std::endl;

                // m->position = cur_position + cur_velocity * delta_t; // explicit method
                m->velocity = cur_velocity + a * delta_t;
                m->velocity = (1 - damping_factor)*m->velocity;
                m->position = cur_position + m->velocity * delta_t; //semi-implicit method

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            auto m1 = s->m1;
            auto m2 = s->m2;

            double l = (m1->start_position - m2->start_position).norm();
            double d = (m1->position - m2->position).norm();

            m1->forces += -(s->k) * (m1->position - m2->position) * (d-l)/d;
            m2->forces += -(s->k) * (m2->position - m1->position) * (d-l)/d;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                auto a = (m->forces + gravity) / m->mass;
                auto cur_position = m->position;              

                double damping_factor = 0.00005;
                m->position = cur_position + (1-damping_factor) * (cur_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = cur_position;
            }
            m->forces = Vector2D(0, 0);
        }
        
    }
}
