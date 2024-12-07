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

        private void ProcessCollisionUsingRTree()
        {
            RTree rTree = new RTree();

            for (int i = 0; i < ptMesh.Vertices.Count; i++)
                rTree.Insert(ptMesh.Vertices[i].ToPoint3d(), i);

            List<int>[] collisionIndices = new List<int>[ptMesh.Vertices.Count];

            for (int i = 0; i < ptMesh.Vertices.Count; i++)
                collisionIndices[i] = new List<int>();

            for (int i = 0; i < ptMesh.Vertices.Count; i++)
                rTree.Search(
                    new Sphere(ptMesh.Vertices[i].ToPoint3d(), CollisionDistance),
                    (sender, args) => { if (i < args.Id) collisionIndices[i].Add(args.Id); });

            for (int i = 0; i < collisionIndices.Length; i++)
            {
                foreach (int j in collisionIndices[i])
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
        }

        private void ProcessBendingResistance()
        {
            int halfedgeCount = ptMesh.Halfedges.Count;
            for (int k = 0; k < halfedgeCount; k += 2)
            {
                // Skip if this edge is naked
                if (ptMesh.Halfedges[k].AdjacentFace == -1 || ptMesh.Halfedges[k + 1].AdjacentFace == -1) continue;

                int i = ptMesh.Halfedges[k].StartVertex;
                int j = ptMesh.Halfedges[k + 1].StartVertex;
                int p = ptMesh.Halfedges[ptMesh.Halfedges[k].PrevHalfedge].StartVertex;
                int q = ptMesh.Halfedges[ptMesh.Halfedges[k + 1].PrevHalfedge].StartVertex;

                Point3d vI = ptMesh.Vertices[i].ToPoint3d();
                Point3d vJ = ptMesh.Vertices[j].ToPoint3d();
                Point3d vP = ptMesh.Vertices[p].ToPoint3d();
                Point3d vQ = ptMesh.Vertices[q].ToPoint3d();

                Vector3d nP = Vector3d.CrossProduct(vJ - vI, vP - vI);
                Vector3d nQ = Vector3d.CrossProduct(vQ - vI, vJ - vI);

                Vector3d planeNormal = (nP + nQ);
                planeNormal.Unitize();

                Point3d planeOrigin = 0.25 * (vI + vJ + vP + vQ);
                Plane plane = new Plane(planeOrigin, planeNormal);
                totalWeightedMoves[i] += BendingResistanceWeight * (plane.ClosestPoint(vI) - vI);
                totalWeightedMoves[j] += BendingResistanceWeight * (plane.ClosestPoint(vJ) - vJ);
                totalWeightedMoves[p] += BendingResistanceWeight * (plane.ClosestPoint(vP) - vP);
                totalWeightedMoves[q] += BendingResistanceWeight * (plane.ClosestPoint(vQ) - vQ);
                totalWeights[i] += BendingResistanceWeight;
                totalWeights[j] += BendingResistanceWeight;
                totalWeights[p] += BendingResistanceWeight;
                totalWeights[q] += BendingResistanceWeight;
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
