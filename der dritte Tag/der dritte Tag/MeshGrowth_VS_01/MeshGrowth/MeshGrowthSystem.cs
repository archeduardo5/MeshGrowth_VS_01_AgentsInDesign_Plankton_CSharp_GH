using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Plankton;
using PlanktonGh;
using Rhino.Geometry;

namespace MeshGrowth
{
    public class MeshGrowthSystem
    {
        public PlanktonMesh ptMesh;

        public bool Grow = false;
        public int MaxVertexCount;
        
        public bool UseRTree;

        public double EdgeLengthConstraintWeight;
        public double CollisionDistance;
        public double CollisionWeight;
        public double BendingResistanceWeight;

        private List<Vector3d> totalWeightedMoves;
        private List<double> totalWeights;

        public MeshGrowthSystem(Mesh startingMesh)
        {
            ptMesh = startingMesh.ToPlanktonMesh();
        }

        public Mesh GetRhinoMesh()
        {
            return ptMesh.ToRhinoMesh();
        }

        public void Update()
        {
            if (Grow) SplitAllLongEdges();

            totalWeightedMoves = new List<Vector3d>();
            totalWeights = new List<double>();

            for (int i = 0; i < ptMesh.Vertices.Count; i++)
            {
                totalWeightedMoves.Add(Vector3d.Zero);
                totalWeights.Add(0.0);
            }

            ProcessEdgeLengthConstraint();
            if (UseRTree) ProcessCollisionUsingRTree();
            else ProcessCollision();
            ProcessBendingResistance();

            UpdateVertexPositionsAndVelicities();
        }

        private void SplitEdge(int edgeIndex)
        {
            int newHalfEdgeIndex = ptMesh.Halfedges.SplitEdge(edgeIndex);

            ptMesh.Vertices.SetVertex(
                ptMesh.Vertices.Count - 1,
                0.5 * (ptMesh.Vertices[ptMesh.Halfedges[edgeIndex].StartVertex].ToPoint3d() + ptMesh.Vertices[ptMesh.Halfedges[edgeIndex + 1].StartVertex].ToPoint3d()));

            if (ptMesh.Halfedges[edgeIndex].AdjacentFace >= 0)
                ptMesh.Faces.SplitFace(newHalfEdgeIndex, ptMesh.Halfedges[edgeIndex].PrevHalfedge);

            if (ptMesh.Halfedges[edgeIndex + 1].AdjacentFace >= 0)
                ptMesh.Faces.SplitFace(edgeIndex + 1, ptMesh.Halfedges[ptMesh.Halfedges[edgeIndex + 1].NextHalfedge].NextHalfedge);
        }

        private void SplitAllLongEdges()
        {
            int halfedgeCount = ptMesh.Halfedges.Count;

            for (int k = 0; k < halfedgeCount; k += 2)
                if (ptMesh.Vertices.Count < MaxVertexCount &&
                    ptMesh.Halfedges.GetLength(k) > 0.99 * CollisionDistance)
                {
                    SplitEdge(k);
                }
        }

        private void ProcessEdgeLengthConstraint()
        {
            int halfedgeCount = ptMesh.Halfedges.Count;

            for (int k = 0; k < halfedgeCount; k += 2)
            {
                PlanktonHalfedge halfedge = ptMesh.Halfedges[k];
                int i = halfedge.StartVertex;
                int j = ptMesh.Halfedges[halfedge.NextHalfedge].StartVertex;

                Vector3d d = ptMesh.Vertices[j].ToPoint3d() - ptMesh.Vertices[i].ToPoint3d();
                if (d.Length > CollisionDistance)
                {
                    Vector3d move = EdgeLengthConstraintWeight * 0.5 * (d);
                    totalWeightedMoves[i] += move;
                    totalWeightedMoves[j] -= move;
                    totalWeights[i] += EdgeLengthConstraintWeight;
                    totalWeights[j] += EdgeLengthConstraintWeight;
                }
            }
        }

        private void ProcessCollision()
        {
            for (int i = 0; i < ptMesh.Vertices.Count; i++)
                for (int j = i + 1; j < ptMesh.Vertices.Count; j++)
                {
                    Vector3d move = ptMesh.Vertices[j].ToPoint3d() - ptMesh.Vertices[i].ToPoint3d();
                    double currentDistance = move.Length;
                    if (currentDistance > CollisionDistance) continue;
                    move *=  0.5 * (currentDistance - CollisionDistance) / currentDistance;
                    totalWeightedMoves[i] += CollisionWeight * move;
                    totalWeightedMoves[j] -= CollisionWeight * move;
                    totalWeights[i] += CollisionWeight;
                    totalWeights[j] += CollisionWeight;
                }
        }

        private void UpdateVertexPositionsAndVelicities()
        {
            for (int i = 0; i < ptMesh.Vertices.Count; i++)
            {
                if (totalWeights[i] == 0) continue;
                PlanktonVertex vertex = ptMesh.Vertices[i];
                Vector3d move = totalWeightedMoves[i] / totalWeights[i];
                ptMesh.Vertices.SetVertex(i, vertex.X + move.X, vertex.Y + move.Y, vertex.Z + move.Z);
            }
        }
    }

}
