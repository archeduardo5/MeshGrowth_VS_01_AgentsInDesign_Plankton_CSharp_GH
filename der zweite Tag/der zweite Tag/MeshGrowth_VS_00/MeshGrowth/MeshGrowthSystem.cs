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

        /*
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
        */
    }

}
