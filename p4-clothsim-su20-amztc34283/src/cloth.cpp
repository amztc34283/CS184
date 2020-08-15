#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  // Store in row-major order (1,2,3,4,5,6,...)
  if (orientation == HORIZONTAL) // y = 1
      for (int i = 0; i < num_height_points; i++) {
          for (int j = 0; j < num_width_points; j++) {
              Vector3D pos = Vector3D(j * (width/float(num_width_points - 1.)), 1., i * (height/float(num_height_points - 1.)));
              PointMass pm = PointMass(pos, false);
              for (int k = 0; k < pinned.size(); k++) {
                  if (pinned[k][0] == j && pinned[k][1] == i) {
                      pm = PointMass(pos, true);
                      break;
                  }
              }
              point_masses.emplace_back(pm);
          }
      }
  else // z = offset
      for (int i = 0; i < num_height_points; i++) {
          for (int j = 0; j < num_width_points; j++) {
              int ran_num = std::rand();
              float offset;
              int half = RAND_MAX / 2;
              if (ran_num < half)
                  offset = -0.001 * (float(ran_num) / float(half));
              else
                  offset = 0.001 * (float(ran_num - half) / float(half));
              Vector3D pos = Vector3D(j * (width/float(num_width_points - 1.)), i * (height/float(num_height_points - 1.)), offset);
              PointMass pm = PointMass(pos, false);
              for (int k = 0; k < pinned.size(); k++) {
                  if (pinned[k][0] == j && pinned[k][1] == i) {
                      pm = PointMass(pos, true);
                      break;
                  }
              }
              point_masses.emplace_back(pm);
          }
      }
  for (int i = 0; i < num_height_points; i++) {
      for (int j = 0; j < num_width_points; j++) {
          PointMass* pm = &point_masses[i * num_width_points + j];
          if (j > 0) {
              // structural with one left
              PointMass* left_pm = &point_masses[i * num_width_points + j - 1];
              springs.emplace_back(Spring(pm, left_pm, STRUCTURAL));
          }
          if (j > 1) {
              // structural with two left
              PointMass* two_left_pm = &point_masses[i * num_width_points + j - 2];
              springs.emplace_back(Spring(pm, two_left_pm, BENDING));
          }
          if (i > 0) {
              // structural with one above
              PointMass* left_pm = &point_masses[(i - 1) * num_width_points + j];
              springs.emplace_back(Spring(pm, left_pm, STRUCTURAL));
          }
          if (i > 1) {
              // structural with two above
              PointMass* left_pm = &point_masses[(i - 2) * num_width_points + j];
              springs.emplace_back(Spring(pm, left_pm, BENDING));
          }
          if (i > 0 && j > 0) {
              // shearing with upper left
              PointMass* left_pm = &point_masses[(i - 1) * num_width_points + j - 1];
              springs.emplace_back(Spring(pm, left_pm, SHEARING));
          }
          if (i > 0 && j < num_width_points - 1) {
              // shearing with upper right
              PointMass* left_pm = &point_masses[(i - 1) * num_width_points + j + 1];
              springs.emplace_back(Spring(pm, left_pm, SHEARING));
          }
      }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D total = Vector3D();
  for (int i = 0; i < external_accelerations.size(); i++) {
      total += external_accelerations[i];
  }
  Vector3D total_force = total * mass;
  for (PointMass& pm: point_masses) {
      pm.forces = total_force;
  }
  for (Spring sp: springs) {
      Vector3D a_to_b = sp.pm_b->position - sp.pm_a->position;
      if (cp->enable_bending_constraints && sp.spring_type == BENDING) {
          // multiply ks by 0.2
          double sp_force = 0.2 * cp->ks * (a_to_b.norm() - sp.rest_length);
          sp.pm_a->forces += sp_force * a_to_b.unit();
          sp.pm_b->forces += -sp_force * a_to_b.unit();
      }
      if (cp->enable_shearing_constraints && sp.spring_type == SHEARING) {
          double sp_force = cp->ks * (a_to_b.norm() - sp.rest_length);
          sp.pm_a->forces += sp_force * a_to_b.unit();
          sp.pm_b->forces += -sp_force * a_to_b.unit();
      }
      if (cp->enable_structural_constraints && sp.spring_type == STRUCTURAL) {
          double sp_force = cp->ks * (a_to_b.norm() - sp.rest_length);
          sp.pm_a->forces += sp_force * a_to_b.unit();
          sp.pm_b->forces += -sp_force * a_to_b.unit();
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& pm: point_masses) {
      if (!pm.pinned) {
          Vector3D new_pos = pm.position + (1 - cp->damping / 100.) * (pm.position - pm.last_position) +
                             delta_t * delta_t * (pm.forces / mass);
          pm.last_position = pm.position;
          pm.position = new_pos;
      }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& pm: point_masses) {
      self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (CollisionObject* co: *collision_objects) {
      for (PointMass& pm: point_masses) {
          co->collide(pm);
      }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring sp: springs) {
      Vector3D direction = sp.pm_b->position - sp.pm_a->position;
      float diff = direction.norm() - 1.1 * sp.rest_length;
      if (sp.pm_a->pinned && !sp.pm_b->pinned && diff > 0) {
          sp.pm_b->position = sp.pm_b->position - direction.unit() * diff;
      } else if (!sp.pm_a->pinned && sp.pm_b->pinned && diff > 0) {
          sp.pm_a->position = sp.pm_a->position + direction.unit() * diff;
      } else if (diff > 0) {
          Vector3D a_pos = sp.pm_a->position;
          Vector3D b_pos = sp.pm_b->position;
          sp.pm_b->position = b_pos - direction.unit() * diff * 0.5;
          sp.pm_a->position = a_pos + direction.unit() * diff * 0.5;
      }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass& pm: point_masses) {
      vector<PointMass*>* pm_vec = map[hash_position(pm.position)];
      if (pm_vec == NULL) {
          map[hash_position(pm.position)] = new vector<PointMass*>();
      }
      map[hash_position(pm.position)]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  vector<PointMass*>* potential_collision = map[hash_position(pm.position)];
  Vector3D total = Vector3D();
  int count = 0;
  if (potential_collision != NULL) {
      for (PointMass *ppm: *potential_collision) {
          if (&pm != ppm) {
              double distance = (pm.position - ppm->position).norm();
              if (distance < 2 * thickness) {
                  Vector3D offset = (pm.position - ppm->position).unit() * (2 * thickness - distance);
                  total += offset;
                  count += 1;
              }
          }
      }
  }
  if (count > 0)
    pm.position += total / count / simulation_steps;
}

int n_choose_k(int n, int k)
{
    int result = 1;
    if (k > n - k) // it cancels out each other
        k = n - k;
    for (int i = 0; i < k; ++i) {
        result *= (n - i);
        result /= (i + 1);
    }
    return result;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w = 3. * width / num_width_points;
  float h = 3. * height / num_height_points;
  float t = max(w, h);
  float new_x = floor(pos.x / w);
  float new_y = floor(pos.y / h);
  float new_z = floor(pos.z / t);
  return n_choose_k(new_x, 1) + n_choose_k(new_x + new_y + 1, 2) + n_choose_k(new_x + new_y + new_z + 2, 3);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
