#include "MVBB.h"
#include "../include/Data.h"
#include <thread_pool.hpp>
// #include <boost/make_shared.hpp>
// #include <boost/shared_ptr.hpp>

using namespace std;

MVBB::MVBB() {}
MVBB::~MVBB(){};

bool MVBB::getQualities(std::string graspPointCloudPath,
                        std::string objectPointCloudPath,
                        std::string transformationsFilePath,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPCFiltered, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &partialObjectPC,
                        pcl::PointCloud<pcl::Normal>::Ptr &objectNormals, 
                        pcl::PointCloud<pcl::Normal>::Ptr &partialObjectNormals, 
                        Eigen::Vector3f &CM) {

        Data *data; data = new Data();
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectPC(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr graspPC(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Quaternionf rotation;
        Eigen::Vector3f translation;
        Eigen::Vector4f min, max;
        Eigen::Matrix4f projection;

        //Pipeline
        if(this->loadPointCloud(graspPointCloudPath, graspPC) != true)
            PCL_ERROR ("Can't read Grasp Point Cloud (.pcd file)\n");
        if(this->loadPointCloud(objectPointCloudPath, objectPC) != true) 
            PCL_ERROR ("Can't read Object Point Cloud (.pcd file)\n");
        this->filterPointCloud(objectPC, objectPCFiltered);
        this->computeNormals(objectPCFiltered, objectNormals, CM);
        int line = data->extractGraspNumber(graspPointCloudPath);
        Eigen::Matrix4f transformation = this->returnTransformation(transformationsFilePath, line);
        this->getHandPCTransformation(graspPC, rotation, translation, min, max, projection, transformation);
        float QTpoints = this->computeQTMpoints(objectPCFiltered, objectNormals, min, max, projection, partialObjectPC, cloudIn, partialObjectNormals);
        float TotalObjectArea = this->getPointCloudArea(objectPC);
        float partialObjectArea = this->getPointCloudArea(partialObjectPC);
        double QTarea = partialObjectArea / TotalObjectArea;
        cout << "\033[1;36mQTpoints for grasp " << line << " is: " << QTpoints << "\033[0m" << endl;
        cout << "\033[1;36mQTarea for grasp " << line << " is: " << QTarea << "\033[0m" << endl;
        cout << " \n";
        // this->visualize(graspPC, partialObjectPC, CM, cloudIn, min, max, rotation, translation, false);
        return true;
}

bool MVBB::computeQualities(std::string graspPointCloudPath,
                            std::string objectPointCloudPath,
                            std::string transformationsFilePath,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPCFiltered, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &partialObjectPC,
                            pcl::PointCloud<pcl::Normal>::Ptr &objectNormals, 
                            pcl::PointCloud<pcl::Normal>::Ptr &partialObjectNormals, 
                            Eigen::Vector3f &CM) {

        Data *data; data = new Data();
        // Multi-threading
        int threads = std::thread::hardware_concurrency();
        ThreadPool multiPool_(threads);
        vector<future<bool>> future_vector;
        
        for(int index = 1; index <= 30; index++) {
            string graspPointCloud = data->changeGraspNumber(graspPointCloudPath, index);
            future_vector.emplace_back(
                multiPool_.enqueue( 
                    &MVBB::getQualities,
                    this,
                    graspPointCloud,
                    objectPointCloudPath,
                    transformationsFilePath,
                    objectPCFiltered,
                    partialObjectPC,
                    objectNormals,
                    partialObjectNormals,
                    CM
                )
            );
        }
        return true;
}

bool MVBB::loadPointCloud(string path, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *pointCloud) == -1) 
        return false;
    else   
        return true;
}

bool MVBB::readPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &C_Object, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    std::fstream obj;
    std::vector<std::vector<double>> points;
    std::vector<std::vector<double>> norms;
    std::vector<double> point;
    std::vector<double> norm;
    double val_point, val_norm;
    points.clear();
    norms.clear();
    obj.open("/home/fernando/PHD/Applications/VRMLtoPCD/build/out/Object.txt");
    //read txt file that contains the points and normals provided by simox
    if(obj.is_open()) {
        do {
            if (obj.eof()) {
                break;
                return false;
            }
            point.clear();
            norm.clear();
            for(int i = 0; i < 6; i++) {
                if(i < 3) 
                    obj >> val_point;
                else 
                    obj >> val_norm;
                point.push_back(val_point);
                norm.push_back(val_norm);
            }
            points.push_back(point);
            norms.push_back(norm);
        } while (!obj.eof());

        //Convert the points into PCL format
        C_Object->resize(points.size());
        normals->resize(norms.size());
        for(size_t j = 0; j < points.size(); j++) {
            C_Object->points[j].x = points.at(j).at(0);
            C_Object->points[j].y = points.at(j).at(1);
            C_Object->points[j].z = points.at(j).at(2);
        }
        for(size_t k = 0; k < norms.size(); k++) {
            normals->points[k].normal[0] = norms.at(k).at(0);
            normals->points[k].normal[0] = norms.at(k).at(1);
            normals->points[k].normal[0] = norms.at(k).at(2);
        }
    }
    else 
        return false;
    return true;
}

void MVBB::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered) {
    
    if (original->points.size() > 900000) {
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ > octree (128.0f);
        octree.setInputCloud(original);
        octree.addPointsFromInputCloud();
        double sampling = 2.8;
        pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
        voxfilter.setInputCloud (original);
        voxfilter.setLeafSize (sampling, sampling, sampling);
        voxfilter.filter(*filtered);
        cout << "Object point cloud filtered with " << filtered->points.size() << " points." << endl;
    }
    else filtered = original;  
}

Eigen::Matrix4f MVBB::returnTransformation(string transformationFilePath, uint line) {
    string sLine = "";
    ifstream read;
    read.open(transformationFilePath);
    uint line_no = 0;
    while (line_no != line && getline(read, sLine))
        ++line_no;
    
    Eigen::Matrix4f transformation;
    transformation = Eigen::Matrix4f::Identity();
    std::vector<float> num;
    std::stringstream ss(sLine);

    float i;
    while (ss >> i) {
        num.push_back(i);
        if (ss.peek() == ',')
            ss.ignore();
    }
    
    transformation << num[0], num[1], num[2], num[3], 
                      num[4], num[5], num[6], num[7], 
                      num[8], num[9], num[10], num[11], 
                      0, 0, 0, 1;
    read.close();
    return transformation;
}

void MVBB::getHandPCTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &handConfiguration, 
                                   Eigen::Quaternionf &bboxRotation,
                                   Eigen::Vector3f &bboxTranslation, 
                                   Eigen::Vector4f &min, 
                                   Eigen::Vector4f &max, 
                                   Eigen::Matrix4f &projection,
                                   Eigen::Matrix4f transformation) {
    Eigen::Matrix4f Tinv;
    Tinv = transformation.inverse();

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedGraspPointCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*handConfiguration, *transformedGraspPointCloud, Tinv);

    ///GRASP MVBB
    //Compute PCA
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*transformedGraspPointCloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*transformedGraspPointCloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    //Transform the original grasp point cloud to the origin where the principal component correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*transformedGraspPointCloud, *cloudPointsProjected, projectionTransform);
    projection = projectionTransform;

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    bboxRotation = bboxQuaternion;
    bboxTranslation = bboxTransform;
    min[0] = minPoint.x;
    min[1] = minPoint.y;
    min[2] = minPoint.z;
    max[0] = maxPoint.x;
    max[1] = maxPoint.y;
    max[2] = maxPoint.z;
    handConfiguration->clear();
    handConfiguration = transformedGraspPointCloud;
}

void MVBB::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                          pcl::PointCloud<pcl::Normal>::Ptr &Normals, 
                          Eigen::Vector3f &CM) {
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*C_Object, centroid);
    CM << centroid[0], centroid[1], centroid[2];
    //Compute normal vectors
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(8);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(C_Object);
    ne.setSearchMethod(tree);
    ne.setKSearch(40);
    ne.setViewPoint(CM[0],CM[1],CM[2]);
    ne.compute(*Normals);
}

float MVBB::computeQTMpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object, 
                             pcl::PointCloud<pcl::Normal>::Ptr normals, 
                             Eigen::Vector4f min, 
                             Eigen::Vector4f max, 
                             Eigen::Matrix4f projection,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsOut, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &pointsIn, 
                             pcl::PointCloud<pcl::Normal>::Ptr &normalsOut) {

    Eigen::Affine3f boxTransform;
    boxTransform.matrix() = projection;
    //Filter Points Outside the CropBox
    std::vector<int> index;
    pcl::CropBox<pcl::PointXYZ> cropFilterOut; // create the filter
    cropFilterOut.setInputCloud (C_Object); //input the object cloud to be filtered
    cropFilterOut.setMin(min);
    cropFilterOut.setMax(max);
    cropFilterOut.setTransform(boxTransform);
    cropFilterOut.setNegative(true);
    cropFilterOut.filter (index);
    pcl::copyPointCloud<pcl::PointXYZ>(*C_Object, index, *pointsOut);
    pcl::copyPointCloud<pcl::Normal>(*normals, index, *normalsOut);
    float pointsOutside = pointsOut->points.size();
    
    //Filter Points Inside the CropBox
    pcl::CropBox<pcl::PointXYZ> cropFilterIn; // create the filter
    cropFilterIn.setInputCloud (C_Object); //input the object cloud to be filtered
    cropFilterIn.setMin(min);
    cropFilterIn.setMax(max);
    cropFilterIn.setTransform(boxTransform);
    cropFilterIn.filter (*pointsIn);
    float pointsInside = pointsIn->points.size() ;
    float QTpoints = pointsOutside / (pointsOutside + pointsInside);
    return QTpoints;
}

float MVBB::getPointCloudArea(pcl::PointCloud<pcl::PointXYZ>::Ptr C_Object) {
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(C_Object);
    n.setInputCloud(C_Object);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr C_ObjectNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*C_Object, *normals, *C_ObjectNormals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(C_ObjectNormals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp.setSearchRadius (10000);

    // Set typical values for the parameters
    gp.setMu(5.0);
    gp.setMaximumNearestNeighbors(2500);
    gp.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp.setMinimumAngle(M_PI / 18); // 10 degrees
    gp.setMaximumAngle(2 * (M_PI / 3)); // 120 degrees
    gp.setNormalConsistency(false);

    // Get result
    gp.setInputCloud(C_ObjectNormals);
    gp.setSearchMethod(tree2);
    gp.reconstruct(triangles);

    //calculate area
    pcl::PointCloud<pcl::PointXYZ> cloudArea;
    cloudArea = *C_Object;

    int index_p1, index_p2, index_p3;
    double x1, x2, x3, y1, y2, y3, z1, z2, z3, a, b, c, q;
    float area = 0.0;

    for(uint i = 0; i < triangles.polygons.size(); i++) {
        index_p1 = triangles.polygons[i].vertices[0];
        index_p2 = triangles.polygons[i].vertices[1];
        index_p3 = triangles.polygons[i].vertices[2];

        x1 = cloudArea.points[index_p1].x;
        y1 = cloudArea.points[index_p1].y;
        z1 = cloudArea.points[index_p1].z;

        x2 = cloudArea.points[index_p2].x;
        y2 = cloudArea.points[index_p2].y;
        z2 = cloudArea.points[index_p2].z;

        x3 = cloudArea.points[index_p3].x;
        y3 = cloudArea.points[index_p3].y;
        z3 = cloudArea.points[index_p3].z;

        //Heron's formula:
        a = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2) + pow((z1 - z2),2));
        b = sqrt(pow((x1 - x3),2) + pow((y1 - y3),2) + pow((z1 - z3),2));
        c = sqrt(pow((x3 - x2),2) + pow((y3 - y2),2) + pow((z3 - z2),2));
        q = (a + b + c) / 2;

        area = area + sqrt(q * (q - a) * (q - b) * (q - c));
    }
    return area;
}

void MVBB::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr handConfigurationPointCloud, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudOut,
                     Eigen::Vector3f centroid, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudIn, 
                     Eigen::Vector4f min,
                     Eigen::Vector4f max, 
                     Eigen::Quaternionf bboxRotation, 
                     Eigen::Vector3f bboxTranslation, 
                     bool fCoordinates) {
    pcl::visualization::PCLVisualizer visualizer("3D Point Cloud Result");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filteredColorOut(pointCloudOut, 0, 255, 0); //Points out the box (green)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filteredColorIn(pointCloudIn, 0, 0, 255);  //Points in the box (blue)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> graspPointCloud (handConfigurationPointCloud, 255, 0, 0); //Grasp (red)
    visualizer.setBackgroundColor(255,255,255); // white background
    visualizer.addPointCloud(pointCloudOut, filteredColorOut, "pointCloudOut");
    visualizer.addPointCloud(pointCloudIn, filteredColorIn, "pointCloudIn");
    visualizer.addPointCloud(handConfigurationPointCloud, graspPointCloud, "graspPointCloud");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "pointCloudOut");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "pointCloudIn");
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "graspPointCloud");
    
    if(fCoordinates) {
        visualizer.addCoordinateSystem(10,"world",0);
        visualizer.addCoordinateSystem(10, centroid[0], centroid[1], centroid[2], "centroid",0);
    }
    visualizer.addCube(bboxTranslation, bboxRotation, max[0] - min[0], max[1] - min[1], max[2] - min[2], "boundingbox", 0);
    visualizer.setRepresentationToWireframeForAllActors(); // see bounding box lines
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "boundingbox");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "boundingbox");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "boundingbox");

    while(!visualizer.wasStopped())
        visualizer.spinOnce();

    visualizer.close();
}