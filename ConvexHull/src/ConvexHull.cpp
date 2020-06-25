#include "ConvexHull.h"

ConvexHull::ConvexHull() {

    Wrenches.clear();
    EWR.clear();
    CM.setZero();
}

ConvexHull::ConvexHull(Eigen::Vector3f centerofmodel) {
    Wrenches.clear();
    EWR.clear();

    CM = centerofmodel;
}

ConvexHull::~ConvexHull(){

}

bool ConvexHull::convertPoints(double *storePointsQHull) {

    for (uint i = 0; i < Wrenches.size(); i++) {
        storePointsQHull[i * 6 + 0] = Wrenches[i].p[0];
        storePointsQHull[i * 6 + 1] = Wrenches[i].p[1];
        storePointsQHull[i * 6 + 2] = Wrenches[i].p[2];
        storePointsQHull[i * 6 + 3] = Wrenches[i].n[0];
        storePointsQHull[i * 6 + 4] = Wrenches[i].n[1];
        storePointsQHull[i * 6 + 5] = Wrenches[i].n[2];
    }

     return true;
}

bool ConvexHull::CreateConvexHull() {

    int Cpoints = (int)Wrenches.size();

    if(Cpoints < 4) {

        std::cout<<"Error: Need at least 4 points (number of points registered: "<< Cpoints <<" )"<<std::endl;
        return false;
    }

    Result.faces.clear();
    Result.vertices.clear();

    int dim = 6;
    coordT* points = new coordT[(dim)*Cpoints];
    boolT ismalloc = False;
    char flags[250];
    FILE* outfile = stdout;
    FILE* errfile = stderr;
    int exitcode;
    int curlong, totlong;
    facetT* facet;
    vertexT* vertex, **vertexp;
    sprintf(flags,"qhull QJ FA");

    convertPoints(points);

    exitcode = qh_new_qhull(dim,Cpoints,points,ismalloc,flags,outfile,errfile);

    if(!exitcode) {
        facetT* facet_list = qh facet_list;
        int convexNumFaces = qh num_facets;
        int convexNumVertex = qh_setsize(qh_facetvertices(facet_list,NULL,false));

        qh_triangulate();
        int convexNumFaces2 = qh num_facets;
        int convexNumVertex2 = qh_setsize(qh_facetvertices(facet_list,NULL,false));
        double pCenter[6];

        for(int u = 0; u < 6; u++) {
            pCenter[u] = 0;
        }

        double pZero[6];

        for(int u = 0; u < 6; u++) {
            pZero[u] = 0;
        }

        int nVertexCount = 0;

        FORALLvertices
        {
            for(int u = 0; u< 6; u++) {
                pCenter[u] += vertex->point[u];
            }
            nVertexCount++;
        }

        if(nVertexCount > 0){
            for(int u = 0; u< 6; u++) {
                pCenter[u] /= (float)nVertexCount;
            }
        }

        for(int u = 0; u< 3; u++) {
            Result.center.p[u] = pCenter[u];
            Result.center.n[u] = pCenter[u+3];
        }

        Mtools::ContactPoint v[6];
        int nIds[6];
        Mtools::TriangleFace6D f;

        int nFacets = 0;
        qh_getarea(qh facet_list);
        Result.volume = qh totvol;
        double p[6];
        p[0] = p[1] = p[2] = p[3]  = p[4]  = p[5] = 0;

        FORALLfacets
        {
            int c = 0;
            f.verts.clear();

            FOREACHvertex_(facet->vertices)
            {
                if(c < 6) {
                    v[c].p[0] = vertex->point[0];
                    v[c].p[1] = vertex->point[1];
                    v[c].p[2] = vertex->point[2];
                    v[c].n[0] = vertex->point[3];
                    v[c].n[1] = vertex->point[4];
                    v[c].n[2] = vertex->point[5];

                    Result.vertices.push_back(v[c]);
                    f.verts.push_back(v[c]);
                    nIds[c] = (int)Result.vertices.size()-1;
                    c++;
                }
                else {
                    std::cout<<"Error, facet with more than 6 vertices not supported ... face number: "<< nFacets<<std::endl;
                }
            }

            f.id[0] = nIds[0];
            f.id[1] = nIds[1];
            f.id[2] = nIds[2];
            f.id[3] = nIds[3];
            f.id[4] = nIds[4];
            f.id[5] = nIds[5];
            f.normal.p[0] = facet->normal[0];
            f.normal.p[1] = facet->normal[1];
            f.normal.p[2] = facet->normal[2];
            f.normal.n[0] = facet->normal[3];
            f.normal.n[1] = facet->normal[4];
            f.normal.n[2] = facet->normal[5];
            f.offset = facet->offset;
            double dist = qh_distnorm(6,pCenter,facet->normal,&(facet->offset));
            f.distNormCenter = dist;
            qh_distplane(pCenter, facet, &dist);
            f.distPlaneCenter = dist;
            dist = qh_distnorm(6,pZero,facet->normal,&(facet->offset));
            f.distNormZero = dist;
            qh_distplane(pZero,facet,&dist);
            f.distPlaneZero = dist;
            Result.faces.push_back(f);
            nFacets++;
        }
    }

    coordT point[dim];
    boolT isoutside;
    realT bestdist;

    for(int i = 0; i < dim; i++)
        point[i] = 0;

    qh_findbestfacet (point, qh_ALL, &bestdist, &isoutside); //'facet' is the closest facet to 'point'
    Quality = (float)(fabs(bestdist));
    mOffset = minOffset();
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort(&curlong, &totlong);
    delete[] points;

    if(curlong || totlong)
        fprintf(errfile, "Qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);

    return true;
}

void ConvexHull::Cwrenches(std::vector<Mtools::ContactPoint> &cpoints, int sides) {

    int count = 0;
    Mtools::ContactPoint temp, p;
    Eigen::VectorXf W (6);
    std::vector<Mtools::ContactPoint>::iterator iter = cpoints.begin();
    std::vector<Eigen::VectorXf> Sw;
    Sw.clear();
    Wrenches.clear();
    EWR.clear();
    float factor = 100;

    while(iter != cpoints.end()) {
        p.p = iter->n;
        temp.p = iter->p - CM;
        temp.n = -(iter->n) ;
        p.n = factor*temp.p.cross(temp.n);
        p.id = iter->id;
        Wrenches.push_back(p);

        W << p.p[0], p.p[1], p.p[2], p.n[0], p.n[1], p.n[2];
        Sw.push_back(W);

        count ++;
        if(count > sides-1) {

            EWR.push_back(Sw);
            Sw.clear();
            count = 0;
        }
        iter++;
    }
}

void ConvexHull::Cwrenches(std::vector<Mtools::ContactPoint> &cpoints) {


    Mtools::ContactPoint temp, p;
    std::vector<Mtools::ContactPoint>::iterator iter = cpoints.begin();
    Wrenches.clear();

    float factor = 100;


    while(iter != cpoints.end()){

        p.p = iter->n;
        temp.p = iter->p - CM;
        temp.n = -(iter->n);
        p.n = factor*temp.p.cross(temp.n);
        p.id = iter->id;
        Wrenches.push_back(p);
        iter++;
    }
}

bool ConvexHull::isForceClosure() {

    std::vector<Mtools::TriangleFace6D>::iterator faceIterator;
    for(faceIterator = Result.faces.begin(); faceIterator != Result.faces.end(); faceIterator++) {
        if(faceIterator->distPlaneZero > 1e-4)
            return false;
    }
    return true;
}

float ConvexHull::minOffset() {

    float fRes = FLT_MAX;
    int nWrongFacets = 0;

    for(size_t i = 0; i < (int)Result.faces.size(); i++) {
        if(Result.faces.at(i).distNormCenter > 0)
            nWrongFacets++;
        else if(-(Result.faces.at(i).distNormCenter) < fRes)
            fRes = -(Result.faces.at(i).distNormCenter);

    }

    if(nWrongFacets > 0)
        std::cout << "WARNING: offset of " << nWrongFacets << " facets > 0 (# of facets: " << Result.faces.size() << ")" << std::endl;

    return fRes;
}


void ConvexHull::ChullCenter() {

    std::vector<Mtools::ContactPoint>::iterator iter;
    Center.p.setZero();
    Center.n.setZero();

    if(Result.vertices.size() == 0) {
        std::cout<< "Error: No vertices..."<<std::endl;
        return;
    }

    for(iter = Result.vertices.begin(); iter != Result.vertices.end(); iter++){
        Center.p += iter->p;
        Center.n += iter->n;
    }

    Center.p /= (float)Result.vertices.size();
    Center.n /= (float)Result.vertices.size();
}

float ConvexHull::minDistCH() {

    ChullCenter();
    Mtools::ContactPoint CCenter;
    CCenter.p.setZero();
    CCenter.n.setZero();

    float minDist = FLT_MAX;
    float dist[6];
    float currentDist2;
    std::vector<Mtools::TriangleFace6D>::iterator faceIter;

    for (faceIter = Result.faces.begin(); faceIter != Result.faces.end(); faceIter++) {
        Mtools::ContactPoint faceCenter;
        faceCenter.p.setZero();
        faceCenter.n.setZero();

        for (int j = 0; j < 6; j++) {
            faceCenter.p += (Result.vertices)[faceIter->id[j]].p;
            faceCenter.n += (Result.vertices)[faceIter->id[j]].n;
        }

        faceCenter.p /= 6.0f;
        faceCenter.n /= 6.0f;
        currentDist2 = 0;

        for (int j = 0; j < 3; j++) {
            dist[j] = (faceCenter.p(j) - CCenter.p(j));
            dist[j + 3] = (faceCenter.n(j) - CCenter.n(j));
            currentDist2 += dist[j] * dist[j];
            currentDist2 += dist[j + 3] * dist[j + 3];
        }

        if (currentDist2 < minDist)
            minDist = currentDist2;
    }
    return sqrtf(minDist);
}





