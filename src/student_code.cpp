#include "student_code.h"
#include "halfEdgeMesh.h"
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
    std::vector<Vector2D> next;
    int size = points.size()-1;
    for (int i = 0; i < size; i++){
      Vector2D cur = (1-t)*points[i] + t*points[i+1];
      next.push_back(cur);
    }
    return next;
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
    std::vector<Vector3D> next;
    int size = points.size()-1;
    for (int i = 0; i < size; i++){
      Vector3D cur = (1-t)*points[i] + t*points[i+1];
      next.push_back(cur);
    }
    return next;
    
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
    if (points.size() == 1){
      return points[0];
    }
    std::vector<Vector3D> next = evaluateStep(points,t);
    return evaluate1D(next,t);
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
    std::vector<Vector3D> next;
    int size = controlPoints.size();
    for (int i = 0; i < size; i++){
      Vector3D cur = evaluate1D(controlPoints[i],u);
      next.push_back(cur);
    }

    return evaluate1D(next,v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D N;

    HalfedgeCIter h = this->halfedge();


    do {
        Vector3D edge1 = h->next()->vertex()->position - h->vertex()->position;
        Vector3D edge2 = h->next()->next()->vertex()->position - h->next()->vertex()->position;

        N += cross(edge1, edge2);
        h = h->twin()->next();
    } while(h !=this->halfedge());
    N.normalize();
    return N;
  }

   EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()){
      return e0;
    }

    HalfedgeIter cur = e0->halfedge();
    HalfedgeIter twin = cur->twin();

    HalfedgeIter e1 = cur->next();
    HalfedgeIter e2 = e1->next();
    HalfedgeIter e3 = twin->next();
    HalfedgeIter e4 = e3->next();

    VertexIter v1 = cur->vertex();
    VertexIter v2 = twin->vertex();
    VertexIter v3 = e2->vertex();
    VertexIter v4 = e4->vertex();

    FaceIter f1 = cur->face();
    FaceIter f2 = twin->face();   

    cur->setNeighbors(e4, twin, v3, e0, f1);
    twin->setNeighbors(e2, cur, v4, e0, f2);
    e1->setNeighbors(cur, e1->twin(), v2, e1->edge(), f1); 
    e2->setNeighbors(e3, e2->twin(), v3, e2->edge(), f2); 
    e3->setNeighbors(twin, e3->twin(), v1, e3->edge(), f2); 
    e4->setNeighbors(e1, e4->twin(), v4, e4->edge(), f1); 

    v1->halfedge() = e3; 
    v2->halfedge() = e1; 
    v3->halfedge() = cur; 
    v4->halfedge() = twin; 

    f1->halfedge() = cur; 
    f2->halfedge() = twin; 


    
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
     // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()){
      return e0->halfedge()->vertex();
    }

    HalfedgeIter cur = e0->halfedge();
    HalfedgeIter twin = cur->twin();

    HalfedgeIter e1 = cur->next();
    HalfedgeIter e2 = e1->next();
    HalfedgeIter e3 = twin->next();
    HalfedgeIter e4 = e3->next();

    VertexIter v1 = cur->vertex();
    VertexIter v2 = twin->vertex();
    VertexIter v3 = e2->vertex();
    VertexIter v4 = e4->vertex();

    FaceIter f1 = cur->face();
    FaceIter f2 = twin->face();

    VertexIter new_v = newVertex();
    new_v->position = (v1->position + v2->position)/2;
    new_v->halfedge() = cur;

    FaceIter f3 = newFace();
    FaceIter f4 = newFace();

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    HalfedgeIter e8 = newHalfedge();
    HalfedgeIter e9 = newHalfedge();
    HalfedgeIter e10 = newHalfedge();
    HalfedgeIter e11 = newHalfedge();
    HalfedgeIter e12 = newHalfedge();
    HalfedgeIter e13 = newHalfedge();

    cur->setNeighbors(e1, twin, new_v, e0, f1);
    twin->setNeighbors(e13, cur, v2, e0, f4);
    e1->setNeighbors(e8, e1->twin(), v2, e1->edge(), f1); 
    e2->setNeighbors(e10, e2->twin(), v3, e2->edge(), f2); 
    e3->setNeighbors(e12, e3->twin(), v1, e3->edge(), f3); 
    e4->setNeighbors(twin, e4->twin(), v4, e4->edge(), f4); 

    e8->setNeighbors(cur, e9, v3, e5, f1); 
    e9->setNeighbors(e2, e8, new_v, e5, f2); 
    e10->setNeighbors(e9, e11, v1, e6, f2); 
    e11->setNeighbors(e3, e10, new_v, e6, f3); 
    e12->setNeighbors(e11, e13, v4, e7, f3); 
    e13->setNeighbors(e4, e12, new_v, e7, f4);

    new_v->halfedge() = cur; 

    v1->halfedge() = e10;
    v2->halfedge() = e1;
    v3->halfedge() = e2;
    v4->halfedge() = e4;

    f1->halfedge() = cur;
    f2->halfedge() = e2;
    f3->halfedge() = e3;
    f4->halfedge() = twin;

    e5->halfedge() = e8;
    e6->halfedge() = e10;
    e7->halfedge() = e12;
    new_v->isNew = true;
    e0->isNew = false;
    e5->isNew = true;
    e6->isNew = false;
    e7->isNew = true;


    return new_v;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    
      v->isNew = false;

    
      Vector3D sum_adj(0, 0, 0);
      int n = 0;

      HalfedgeIter cur = v->halfedge();
      do {
        sum_adj += cur->twin()->vertex()->position;
        n++;
        cur = cur->twin()->next();
      } while (cur != v->halfedge());

      double u;
      if (n == 3) {
        u = 3.0 / 16;
      } else {
        u = 3.0 / (8 * n);
      }
      v->newPosition = (1 - n * u) * v->position + u * sum_adj;
    }

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      Vector3D A = e->halfedge()->vertex()->position;
      Vector3D B = e->halfedge()->twin()->vertex()->position;
      Vector3D C = e->halfedge()->next()->next()->vertex()->position;
      Vector3D D = e->halfedge()->twin()->next()->next()->vertex()->position;

      e->newPosition = 3.0 / 8 * (A + B) + 1.0 / 8 * (C + D);
      e->isNew = false; 
    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    std::vector<EdgeIter> originalEdges;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
        originalEdges.push_back(e);
    }

    for (EdgeIter e : originalEdges) {
        VertexIter newVertex = mesh.splitEdge(e);
        newVertex->isNew = true; 
        newVertex->position = e->newPosition; 
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew && (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew)) {
        mesh.flipEdge(e);
      }
    }
    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      if (!v->isNew) {
        v->position = v->newPosition;
      }
    }

      
  }
}
