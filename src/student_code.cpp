/*
 * Student solution for UC Berkeley Project 2
 *
 * Implemented by ____ on ____.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CGL {

    void BezierPatch::preprocess() {
        // TODO Part 1.
        // TODO If you use the matrix form for Bezier patch evaluation, you will need to
        // TODO compute your matrices based on the 16 control points here. 
        // TODO You will also need to define your matrices
        // TODO as member variables in the "BezierPatch" class.
        // TODO If you use De Casteljau's recursive algorithm, you will not need to do anything here.
        double data[16] = { 1,  0,  0,  0,
                           -3,  3,  0,  0,
                            3, -6,  3,  0,
                           -1,  3, -3,  1};
        double dataX[16];
        double dataY[16];
        double dataZ[16];
        for (int column = 0; column < 4; column++){
            for (int row = 0; row < 4; row++){
                dataX[column*4 + row] = controlPoints[row][column].x;
                dataY[column*4 + row] = controlPoints[row][column].y;
                dataZ[column*4 + row] = controlPoints[row][column].z;
            }
        }
        Matrix4x4 M = Matrix4x4(data);
        Matrix4x4 MTranspose = M.T();
        Matrix4x4 xMatrix = Matrix4x4(dataX);
        Matrix4x4 yMatrix = Matrix4x4(dataY);
        Matrix4x4 zMatrix = Matrix4x4(dataZ);
        bezierMatrixX = (M*xMatrix)*(MTranspose);
        bezierMatrixY = (M*yMatrix)*(MTranspose);
        bezierMatrixZ = (M*zMatrix)*(MTranspose);
    }

    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // TODO Returns the 3D point whose parametric coordinates are (u, v) on the Bezier patch.
        // TODO Note that both u and v are within [0, 1].
        Vector4D vVector = Vector4D(1, v, pow(v, 2), pow(v, 3));
        Vector4D uVector = Vector4D(1, u, pow(u, 2), pow(u, 3));
        double x = dot(uVector,(bezierMatrixX*vVector));
        double y = dot(uVector,(bezierMatrixY*vVector));
        double z = dot(uVector,(bezierMatrixZ*vVector));
        return Vector3D(x, y, z);
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        // TODO Tessellate the given Bezier patch into triangles uniformly on a 8x8 grid(8x8x2=128 triangles) in parameter space.
        // TODO You will call your own evaluate function here to compute vertex positions of the tessellated triangles.
        // TODO The "addTriangle" function inherited from the "BezierPatchLoader" class may help you add triangles to the output mesh. 
        Vector3D bezierPoints[9][9];
        for (int v = 0; v <= 8; v++){
            for (int u = 0; u <= 8; u++) {
                bezierPoints[v][u] = BezierPatch::evaluate(u/8.0, v/8.0);
            } 
        }
        for (int column = 0; column < 8; column++){
            for (int row = 0; row < 8; row++){
                BezierPatch::addTriangle(mesh, bezierPoints[column][row],
                                               bezierPoints[column][row + 1],
                                               bezierPoints[column + 1][row]);
                BezierPatch::addTriangle(mesh, bezierPoints[column+1][row], 
                                               bezierPoints[column][row + 1],
                                               bezierPoints[column+1][row + 1]);
            }
        }
    }

    Vector3D Vertex::normal(void) const
    // TODO Part 2.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
     
        //TODO Compute and return the area-weighted unit normal.
        Vector3D n(0,0,0);
        HalfedgeCIter h = halfedge();
        Vector3D vPosition = h->vertex()->position;
        h = h->twin();
        VertexCIter firstNeighbor = h->vertex();
        Vector3D currVector = firstNeighbor->position - vPosition;
        Vector3D nextVector = firstNeighbor->position - vPosition;
        do {
            currVector = nextVector;
            h = h->next()->twin();
            nextVector = h->vertex()->position - vPosition;
            Vector3D normal = cross(currVector, nextVector)*-1;
            n += normal;
        } while (h->vertex() != firstNeighbor);
        return n/n.norm();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 3.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.

        if (!e0->isBoundary()){
            
            //INITIALIZATION
            //Halfedges
            HalfedgeIter h = e0->halfedge();
            HalfedgeIter hTwin = h->twin();
            HalfedgeIter hNext = h->next();
            HalfedgeIter hNextNext = hNext->next();
            HalfedgeIter hTwinNext = hTwin->next();
            HalfedgeIter hTwinNextNext = hTwinNext->next();

            //Vertices
            VertexIter topV = hTwin->vertex();
            VertexIter botV = h->vertex();
            VertexIter leftV = hNextNext->vertex();
            VertexIter rightV = hTwinNextNext->vertex();

            //Faces
            FaceIter hFace = h->face();
            FaceIter htFace = hTwin->face();

            //REASSIGNMENTS

            //Halfedges
            h->setNeighbors(hTwinNextNext, hTwin, leftV, h->edge(), hFace);
            hTwinNextNext->setNeighbors(hNext, hTwinNextNext->twin(), rightV, hTwinNextNext->edge(), hFace);
            hNext->setNeighbors(h, hNext->twin(), topV, hNext->edge(), hFace);

            hTwin->setNeighbors(hNextNext, h, rightV, hTwin->edge(), htFace);
            hNextNext->setNeighbors(hTwinNext, hNextNext->twin(), leftV, hNextNext->edge(), htFace);
            hTwinNext->setNeighbors(hTwin, hTwinNext->twin(), botV, hTwinNext->edge(), htFace);

            //faces
            hFace->halfedge() = h;
            htFace->halfedge() = hTwin;

            //Vertices
            topV->halfedge() = hNext;
            rightV->halfedge() = hNextNext;
            botV->halfedge() = hTwinNext;
            rightV->halfedge() = hTwinNextNext;

            
        }
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

        //Declarations
        //New Vertex
        if (e0->isBoundary()){
            return VertexIter();
        }

        //INITIALIZATION

        //Halfedges
        HalfedgeIter h = e0->halfedge();
        HalfedgeIter hTwin = h->twin();
        HalfedgeIter hNext = h->next();
        HalfedgeIter hNextNext = hNext->next();
        HalfedgeIter hTwinNext = hTwin->next();
        HalfedgeIter hTwinNextNext = hTwinNext->next();

        HalfedgeIter e1HE0 = newHalfedge();
        HalfedgeIter e1HE1 = newHalfedge();
        HalfedgeIter e2HE0 = newHalfedge();
        HalfedgeIter e2HE1 = newHalfedge();
        HalfedgeIter e3HE0 = newHalfedge();
        HalfedgeIter e3HE1 = newHalfedge();

        //Edges
        EdgeIter e1 = newEdge();
        EdgeIter e2 = newEdge();
        EdgeIter e3 = newEdge();

        //Faces
        FaceIter f0 = h->face();
        FaceIter f1 = newFace();
        FaceIter f2 = newFace();
        FaceIter f3 = hTwin->face();

        //Vertices
        VertexIter topV = hNext->vertex();
        VertexIter leftV = hNextNext->vertex();
        VertexIter botV = hTwinNext->vertex();
        VertexIter rightV = hTwinNextNext->vertex();

        //midPoint
        VertexIter midPoint = newVertex();
        h->vertex() = midPoint;
        midPoint->halfedge() = h;
        midPoint->position = (topV->position + botV->position)/2;

        //REASSIGNMENTS

        //face0
        h->setNeighbors(hNext, hTwin, midPoint, e0, f0);
        hNext->setNeighbors(e1HE0, hNext->twin(), topV, hNext->edge(), f0);
        e1HE0->setNeighbors(h, e1HE1, leftV, e1, f0);
        f0->halfedge() = h;
        e0->halfedge() = h;

        //face1
        e1HE1->setNeighbors(hNextNext, e1HE0, midPoint, e1, f1);
        hNextNext->setNeighbors(e2HE0, hNextNext->twin(), leftV, hNextNext->edge(), f1);
        e2HE0->setNeighbors(e1HE1, e2HE1, botV, e2, f1);
        f1->halfedge() = e1HE1;
        e1->halfedge() = e1HE1;

        //face2
        e2HE1->setNeighbors(hTwinNext, e2HE0, midPoint, e2, f2);
        hTwinNext->setNeighbors(e3HE1, hTwinNext->twin(), botV, hTwinNext->edge(), f2);
        e3HE1->setNeighbors(e2HE1, e3HE0, rightV, e3, f2);
        f2->halfedge() = e2HE1;
        e2->halfedge() = e2HE1;

        //face3
        e3HE0->setNeighbors(hTwinNextNext, e3HE1, midPoint, e3, f3);
        hTwinNextNext->setNeighbors(hTwin, hTwinNextNext->twin(), rightV, hTwinNextNext->edge(), f3);
        hTwin->setNeighbors(e3HE0, h, topV, e0, f3);
        f3->halfedge() = e3HE0;
        e3->halfedge() = e3HE0;

        e0->isNew = true;
        e1->isNew = true;
        e2->isNew = true;
        e3->isNew = true;
        midPoint->isNew = true;

        e0->flipped = false;
        e1->flipped = false;
        e2->flipped = false;
        e3->flipped = false;


        if (!leftV->isNew){
            e1->flipped = true;
        }

        if (!rightV->isNew) {
            e3->flipped = true;
        }

        return midPoint;
    }

    void MeshResampler::upsample(HalfedgeMesh& mesh)
    // TODO Part 5.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.


        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)


        // TODO Now flip any new edge that connects an old and new vertex.


        // TODO Finally, copy the new vertex positions into final Vertex::position.


        //-------MY CODE STARTS HERE-------

        //Update vertex positions
        for (VertexIter v = mesh.verticesBegin(); v!= mesh.verticesEnd(); v++){
            v->isNew = false;
            Vector3D neighborSum = Vector3D(0, 0 ,0);
            Vector3D oriPosition = v->position;
            HalfedgeIter h = v->halfedge()->twin();
            VertexIter firstNeighbor = h->vertex();
            double numNeighbors = 0;
            double u = 0;
            do{
                neighborSum += h->vertex()->position;
                h = h->next()->twin();
                numNeighbors += 1;
            }while (h->vertex() != firstNeighbor);

            u = 3.0/(8.0*numNeighbors);
            if (numNeighbors == 3) {
                u = 3.0/16.0;
            }
            v->newPosition = (1.0 - numNeighbors*u)*oriPosition + u*neighborSum;

        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
            e->flipped = false;
            e->isNew = false;
            HalfedgeIter h = e->halfedge();
            VertexIter topV = h->next()->next()->vertex();
            VertexIter botV = h->twin()->next()->next()->vertex();
            VertexIter rightV = h->twin()->vertex();
            VertexIter leftV = h->vertex();
            e->newPosition = (3.0/8.0)*(leftV->position + rightV->position) + (1.0/8.0)*(topV->position + botV->position);   
        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
            if(!e->isNew){
                VertexIter newVertex = mesh.splitEdge(e);
                newVertex->newPosition = e->newPosition;
            }
        }

        for (VertexIter v = mesh.verticesBegin(); v!= mesh.verticesEnd(); v++){
            v->position = v->newPosition;
        }

        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
            if (e->flipped) {
                mesh.flipEdge(e);
            }
        }


    }

    // TODO Part 6.
    // TODO There's also some code you'll need to complete in "Shader/frag" file.

}
