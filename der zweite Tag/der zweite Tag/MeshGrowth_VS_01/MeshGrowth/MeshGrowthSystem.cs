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
        public PlanktonMesh Mesh;

        public bool Grow = false;
        public int MaxVertexCount;

        public bool UseRTree;

        public double EdgeLengthConstraintWeight;
        public double CollisionDistance;
        public double CollisionWeight;
        public double BendingResistanceWeight;

        private List<Vector3d> totalWeightedMoves;
        private List<double> totalWeights;

        public MeshGrowthSystem(PlanktonMesh startingMesh)
        {
            Mesh = startingMesh;
        }

        public void Update()
        {
            if (Grow) SplitAllLongEdges();

            totalWeightedMoves = new List<Vector3d>();
            totalWeights = new List<double>();

            for (int i = 0; i < Mesh.Vertices.Count; i++)
            {
                totalWeightedMoves.Add(Vector3d.Zero);
                totalWeights.Add(0.0);
            }

            ProcessEdgeLengthConstraint();
            ProcessCollision();

            UpdateVertexPositionsAndVelicities();
        }


        private void SplitEdge(int edgeIndex)
        {
            int newHalfEdgeIndex = Mesh.Halfedges.SplitEdge(edgeIndex);

            Mesh.Vertices.SetVertex(
                Mesh.Vertices.Count - 1,
                0.5 * (Mesh.Vertices[Mesh.Halfedges[edgeIndex].StartVertex].ToPoint3d() + Mesh.Vertices[Mesh.Halfedges[edgeIndex + 1].StartVertex].ToPoint3d()));

            if (Mesh.Halfedges[edgeIndex].AdjacentFace >= 0)
                Mesh.Faces.SplitFace(newHalfEdgeIndex, Mesh.Halfedges[edgeIndex].PrevHalfedge);

            if (Mesh.Halfedges[edgeIndex + 1].AdjacentFace >= 0)
                Mesh.Faces.SplitFace(edgeIndex + 1, Mesh.Halfedges[Mesh.Halfedges[edgeIndex + 1].NextHalfedge].NextHalfedge);
        }


        private void SplitAllLongEdges()
        {
            int halfedgeCount = Mesh.Halfedges.Count;

            for (int k = 0; k < halfedgeCount; k += 2)
                if (Mesh.Vertices.Count < MaxVertexCount &&
                    Mesh.Halfedges.GetLength(k) > 0.99 * CollisionDistance)
                {
                    SplitEdge(k);
                }
        }

        private void ProcessEdgeLengthConstraint()
        {
            int halfedgeCount = Mesh.Halfedges.Count;

            for (int k = 0; k < halfedgeCount; k += 2)
            {
                PlanktonHalfedge halfedge = Mesh.Halfedges[k];
                int i = halfedge.StartVertex;
                int j = Mesh.Halfedges[halfedge.NextHalfedge].StartVertex;

                Vector3d d = Mesh.Vertices[j].ToPoint3d() - Mesh.Vertices[i].ToPoint3d();
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
            for (int i = 0; i < Mesh.Vertices.Count; i++)
                for (int j = i + 1; j < Mesh.Vertices.Count; j++)
                {
                    Vector3d move = Mesh.Vertices[j].ToPoint3d() - Mesh.Vertices[i].ToPoint3d();
                    double currentDistance = move.Length;
                    if (currentDistance > CollisionDistance) continue;
                    move *= CollisionWeight * 0.5 * (currentDistance - CollisionDistance) / currentDistance;
                    totalWeightedMoves[i] += move;
                    totalWeightedMoves[j] -= move;
                    totalWeights[i] += CollisionWeight;
                    totalWeights[j] += CollisionWeight;
                }
        }

        private void UpdateVertexPositionsAndVelicities()
        {
            for (int i = 0; i < Mesh.Vertices.Count; i++)
            {
                if (totalWeights[i] == 0) continue;
                PlanktonVertex vertex = Mesh.Vertices[i];
                Vector3d move = totalWeightedMoves[i] / totalWeights[i];
                Mesh.Vertices.SetVertex(i, vertex.X + move.X, vertex.Y + move.Y, vertex.Z + move.Z);
            }
        }
    }

}
