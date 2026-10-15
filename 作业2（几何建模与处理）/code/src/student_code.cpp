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
    vector<Vector2D> result;
    for (size_t i = 0; i + 1 < points.size(); i++)
      result.push_back((1 - t) * points[i] + t * points[i + 1]);

    return result;
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
    vector<Vector3D> result;
    for (size_t i = 0; i + 1 < points.size(); i++)
      result.push_back((1 - t) * points[i] + t * points[i + 1]);

    return result;
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
    vector<Vector3D> level = points;
    while (level.size() > 1)
      level = evaluateStep(level, t);

    return level[0];
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
    vector<Vector3D> rows;
    for (const auto& row : controlPoints)
      rows.push_back(evaluate1D(row, u));

    return evaluate1D(rows, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    Vector3D sum;
    HalfedgeCIter h = halfedge();
    do
    {
      if (!h->isBoundary())
      {
        Vector3D a = h->next()->vertex()->position - position;
        Vector3D b = h->next()->next()->vertex()->position - position;
        sum += cross(a, b);
      }
      h = h->twin()->next();
    }
    while (h != halfedge());

    return sum.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    if (e0->isBoundary())
      return e0;

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    VertexIter a = h0->vertex();
    VertexIter b = h3->vertex();
    VertexIter c = h2->vertex();
    VertexIter d = h5->vertex();
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    h0->setNeighbors(h5, h3, c, e0, f0);
    h1->setNeighbors(h0, h1->twin(), b, h1->edge(), f0);
    h2->setNeighbors(h4, h2->twin(), c, h2->edge(), f1);
    h3->setNeighbors(h2, h0, d, e0, f1);
    h4->setNeighbors(h3, h4->twin(), a, h4->edge(), f1);
    h5->setNeighbors(h1, h5->twin(), d, h5->edge(), f0);

    a->halfedge() = h4;
    b->halfedge() = h1;
    c->halfedge() = h0;
    d->halfedge() = h3;
    e0->halfedge() = h0;
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    HalfedgeIter h0 = e0->halfedge();
    if (h0->isBoundary())
      h0 = h0->twin();

    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    VertexIter a = h0->vertex();
    VertexIter b = h3->vertex();
    VertexIter c = h2->vertex();
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    VertexIter m = newVertex();
    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    FaceIter f2 = newFace();
    HalfedgeIter h6 = newHalfedge();
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();
    m->position = (a->position + b->position) / 2;
    m->isNew = true;
    e0->isNew = false;
    e1->isNew = false;
    e2->isNew = true;

    h0->setNeighbors(h8, h3, a, e0, f0);
    h1->setNeighbors(h9, h1->twin(), b, h1->edge(), f2);
    h2->setNeighbors(h0, h2->twin(), c, h2->edge(), f0);
    h7->setNeighbors(h1, h6, m, e1, f2);
    h8->setNeighbors(h2, h9, m, e2, f0);
    h9->setNeighbors(h7, h8, c, e2, f2);

    a->halfedge() = h0;
    b->halfedge() = h1;
    c->halfedge() = h2;
    m->halfedge() = h7;
    e0->halfedge() = h0;
    e1->halfedge() = h7;
    e2->halfedge() = h8;
    f0->halfedge() = h0;
    f2->halfedge() = h7;

    if (f1->isBoundary())
    {
      // Keep the existing boundary halfedge rooted at b to preserve its predecessor.
      h6->setNeighbors(h3->next(), h0, m, e0, f1);
      h3->setNeighbors(h6, h7, b, e1, f1);
      h0->twin() = h6;
      h7->twin() = h3;
      b->halfedge() = h3;
      m->halfedge() = h6;
      f1->halfedge() = h3;
    }
    else
    {
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      VertexIter d = h5->vertex();

      EdgeIter e3 = newEdge();
      FaceIter f3 = newFace();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      e3->isNew = true;

      h3->setNeighbors(h4, h0, m, e0, f1);
      h4->setNeighbors(h11, h4->twin(), a, h4->edge(), f1);
      h5->setNeighbors(h6, h5->twin(), d, h5->edge(), f3);
      h6->setNeighbors(h10, h7, b, e1, f3);
      h10->setNeighbors(h5, h11, m, e3, f3);
      h11->setNeighbors(h3, h10, d, e3, f1);

      d->halfedge() = h5;
      e3->halfedge() = h10;
      f1->halfedge() = h3;
      f3->halfedge() = h6;
    }

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
    {
      bool boundary = v->isBoundary();
      Vector3D sum;
      size_t n = 0;
      HalfedgeIter h = v->halfedge();
      do
      {
        if (!boundary || h->edge()->isBoundary())
        {
          sum += h->twin()->vertex()->position;
          n++;
        }
        h = h->twin()->next();
      }
      while (h != v->halfedge());

      if (boundary)
        v->newPosition = 0.75 * v->position + 0.125 * sum;
      else
      {
        double u = n == 3 ? 3.0 / 16 : 3.0 / (8 * n);
        v->newPosition = (1 - n * u) * v->position + u * sum;
      }
      v->isNew = false;
    }

    vector<EdgeIter> edges;
    edges.reserve(mesh.nEdges());
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
    {
      HalfedgeIter h = e->halfedge();
      Vector3D a = h->vertex()->position;
      Vector3D b = h->twin()->vertex()->position;
      if (e->isBoundary())
        e->newPosition = (a + b) / 2;
      else
      {
        Vector3D c = h->next()->next()->vertex()->position;
        Vector3D d = h->twin()->next()->next()->vertex()->position;
        e->newPosition = 0.375 * (a + b) + 0.125 * (c + d);
      }
      e->isNew = false;
      edges.push_back(e);
    }

    for (EdgeIter e : edges)
    {
      VertexIter v = mesh.splitEdge(e);
      v->newPosition = e->newPosition;
    }

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
    {
      HalfedgeIter h = e->halfedge();
      if (e->isNew && h->vertex()->isNew != h->twin()->vertex()->isNew)
        mesh.flipEdge(e);
    }

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
      v->position = v->newPosition;
  }
}
