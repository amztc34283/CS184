#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> next_lvl(points.size() - 1);
    for (int i = 0; i < next_lvl.size(); i++)
        next_lvl[i] = (1 - t) * points[i] + t * points[i + 1];
    return next_lvl;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> next_lvl(points.size() - 1);
    for (int i = 0; i < next_lvl.size(); i++)
        next_lvl[i] = (1 - t) * points[i] + t * points[i + 1];
    return next_lvl;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> ptr = points;
    for (int i = 0; i < points.size() - 1; i++)
         ptr = evaluateStep(ptr, t);
    return ptr[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> vertical_pts(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); i++)
        vertical_pts[i] = evaluate1D(controlPoints[i], u);
    return evaluate1D(vertical_pts, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D result = Vector3D();
    HalfedgeCIter start = halfedge();
    HalfedgeCIter ptr = start;
    do {
       Vector3D original_pt_pos = ptr->vertex()->position;
       Vector3D next_pt_pos = ptr->next()->vertex()->position;
       Vector3D next_twin_pt_pos = ptr->next()->twin()->vertex()->position;
       result += cross(next_pt_pos - original_pt_pos, next_twin_pt_pos - original_pt_pos);
       ptr = ptr->twin()->next();
    } while (ptr != start);
    return result.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary())
          return e0;
      // Part 4.
      // This method should flip the given edge and return an iterator to the flipped edge.
      // Pass by reference was creating a bug here
      // Get the half edge of this edge pointed by e0 and its twin
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();
      // Change halfedges of vertices b and c
      VertexIter v_a = bc->next()->next()->vertex();
      VertexIter v_b = bc->vertex();
      VertexIter v_c = cb->vertex();
      VertexIter v_d = cb->next()->next()->vertex();
      v_b->halfedge() = cb->next();
      v_c->halfedge() = bc->next();
      // Change halfedges of the faces
      FaceIter f_b = bc->face();
      FaceIter f_c = cb->face();
      f_b->halfedge() = bc->next();
      f_c->halfedge() = cb->next();
      // Update next and face of ab dc ca bd
      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();
      ca->next() = bc;
      ab->next() = bd;
      bd->next() = cb;
      dc->next() = ca;
      ca->face() = f_b;
      ab->face() = f_c;
      bd->face() = f_c;
      dc->face() = f_b;
      // Update next, vertex, and face of bc and cb
      bc->next() = dc;
      cb->next() = ab;
      bc->vertex() = v_a;
      cb->vertex() = v_d;
      bc->face() = f_b;
      cb->face() = f_c;
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter bm = e0->halfedge(); // original bc
      HalfedgeIter cm = bm->twin(); // original cb
      // create new elements
      VertexIter vtx_m = newVertex();
      FaceIter new_fc_1 = newFace();
      FaceIter new_fc_2 = newFace();
      EdgeIter edge_am = newEdge();
      EdgeIter edge_dm = newEdge();
      EdgeIter edge_cm = newEdge();
      edge_am->isNew = true;
      edge_dm->isNew = true;
      // reuse original edge as bm
      EdgeIter edge_bm = e0;
      FaceIter fc_1 = bm->face();
      FaceIter fc_2 = cm->face();
      VertexIter vtx_b = bm->vertex();
      VertexIter vtx_c = cm->vertex();
      VertexIter vtx_a = bm->next()->next()->vertex();
      VertexIter vtx_d = cm->next()->next()->vertex();

      // reuse bc and cb halfedges for bm and cm
      HalfedgeIter mc = newHalfedge();
      HalfedgeIter mb = newHalfedge();
      HalfedgeIter am = newHalfedge();
      HalfedgeIter ma = newHalfedge();
      HalfedgeIter md = newHalfedge();
      HalfedgeIter dm = newHalfedge();
      HalfedgeIter ca = bm->next();
      HalfedgeIter ab = ca->next();
      HalfedgeIter bd = cm->next();
      HalfedgeIter dc = bd->next();

      // change existing stuff
      edge_bm->halfedge() = bm;
      fc_1->halfedge() = bm;
      fc_2->halfedge() = cm;
      vtx_b->halfedge() = bm;
      vtx_c->halfedge() = cm;
      // change bm and cm stuff
      cm->edge() = edge_cm;
      bm->next() = ma;
      cm->next() = md;
      bm->twin() = mb;
      cm->twin() = mc;

      vtx_m->halfedge() = ma;

      // handle vertex assignment
      ma->vertex() = vtx_m;
      am->vertex() = vtx_a;
      mc->vertex() = vtx_m;
      mb->vertex() = vtx_m;
      md->vertex() = vtx_m;
      dm->vertex() = vtx_d;

      // handle face assignment
      mc->face() = new_fc_1;
      ca->face() = new_fc_1;
      new_fc_1->halfedge() = mc;
      mb->face() = new_fc_2;
      bd->face() = new_fc_2;
      new_fc_2->halfedge() = mb;
      am->face() = new_fc_1;
      ma->face() = fc_1;
      md->face() = fc_2;
      dm->face() = new_fc_2;

      // handle edge assignment
      mc->edge() = edge_cm;
      edge_cm->halfedge() = mc;
      mb->edge() = edge_bm;
      am->edge() = edge_am;
      ma->edge() = edge_am;
      edge_am->halfedge() = ma;
      md->edge() = edge_dm;
      dm->edge() = edge_dm;
      edge_dm->halfedge() = md;

      // handle next assignment
      mc->next() = ca;
      ca->next() = am;
      am->next() = mc;
      mb->next() = bd;
      bd->next() = dm;
      dm->next() = mb;
      ma->next() = ab;
      ab->next() = bm;
      bm->next() = ma;
      md->next() = dc;
      dc->next() = cm;
      cm->next() = md;

      // handle twin assignment
      am->twin() = ma;
      ma->twin() = am;
      bm->twin() = mb;
      mb->twin() = bm;
      cm->twin() = mc;
      mc->twin() = cm;
      dm->twin() = md;
      md->twin() = dm;

      // Calculate midpoint for vtx_m
      vtx_m->position = (vtx_b->position + vtx_c->position) / 2;

      return vtx_m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
      for (VertexIter e = mesh.verticesBegin(); e != mesh.verticesEnd(); e++)
      {
          Vector3D sum = Vector3D();
          HalfedgeIter first = e->halfedge();
          HalfedgeIter ptr = first;
          do {
              sum += ptr->twin()->vertex()->position;
              ptr = ptr->twin()->next();
          } while(ptr != first);
          Size degree = e->degree();
          float u;
          if ((int) degree == 3) {
              u = (float) 3/ (float) 16;
          } else {
              u = (float) 3/ (float) (8 * degree);
          }
          e->newPosition = (1 - (float) degree * u) * e->position + u * sum;
          e->isNew = false;
      }
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
      {
          HalfedgeIter AB = e->halfedge();
          HalfedgeIter BA = AB->twin();
          HalfedgeIter DA = AB->next()->next();
          HalfedgeIter CB = BA->next()->next();
          e->newPosition = ((float) 3 / (float) 8) * (AB->vertex()->position + BA->vertex()->position)
                           + ((float) 1 / (float) 8) * (CB->vertex()->position + DA->vertex()->position);
          e->isNew = false;

      }
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
      {
          if (e->isNew)
              break;
          VertexIter vtx = mesh.splitEdge(e);
          vtx->newPosition = e->newPosition;
          vtx->isNew = true;
      }
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
      {
          if ((e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew && e->isNew) ||
              (!e->halfedge()->vertex()->isNew && e->halfedge()->twin()->vertex()->isNew && e->isNew))
              // Suspect flipedge is not working perfectly
              mesh.flipEdge(e);
      }
      for (VertexIter e = mesh.verticesBegin(); e != mesh.verticesEnd(); e++)
          e->position = e->newPosition;
      return;
  }
}