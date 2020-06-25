#include "MVBB.h"

using namespace std;

MVBB::MVBB() {}
MVBB::~MVBB(){}

// wrench *wrc; wrc = new wrench();

bool MVBB::getQualities(std::string graspPointCloudPath,
                        std::string objectPointCloudPath,
                        std::string transformationsFilePath,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPCFiltered, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &partialObjectPC,
                        pcl::PointCloud<pcl::Normal>::Ptr &objectNormals, 
                        pcl::PointCloud<pcl::Normal>::Ptr &partialObjectNormals, 
                        Eigen::Vector3f &CM) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr objectPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr graspPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Quaternionf rotation;
    Eigen::Vector3f translation;
    Eigen::Vector4f min, max;
    Eigen::Matrix4f projection;
  
    //  if change object, change path
    const char *absPath1 = "/home/fernando/PHD/Experiments/grasps/CookieBox/box30.xml";
    const char *absPath2 = "/home/fernando/PHD/Experiments/grasps/BabyBottle/GraspTransforms30.txt";
    const char *absPath3 = "/home/fernando/PHD/Experiments/grasps/CookieBox/GraspsQuality30.txt";
    //const char *absPath4 = "/home/fernando/PHD/Experiments/grasps/Box/box200.xml";
    //const char *absPath5 = "/home/fernando/PHD/Experiments/grasps/Box/SortQualities1.txt";

    //Pipeline
    loadPointCloud(graspPointCloudPath, graspPC);
    loadPointCloud(objectPointCloudPath, objectPC);
    filterPointCloud(objectPC, objectPCFiltered);
    computeNormals(objectPCFiltered, objectNormals, CM);
    int line = extractGraspNumber(graspPointCloudPath);
    Eigen::Matrix4f transformation = returnTransformation(transformationsFilePath, line);
    getHandPCTransformation(graspPC, rotation, translation, min, max, projection, transformation);
    //getGraspQuality(absPath1, absPath3);
    //getTransforms(absPath1, absPath2);
    //QualitySort(absPath4, absPath5);
    float QTpoints = computeQTMpoints(objectPCFiltered, objectNormals, min, max, projection, partialObjectPC, cloudIn, partialObjectNormals);
    float TotalObjectArea = getPointCloudArea(objectPC);
    float partialObjectArea = getPointCloudArea(partialObjectPC);
    double QTarea = partialObjectArea / TotalObjectArea;
    cout << "QTpoints for grasp " << line << " is: " << QTpoints << endl;
    cout << "QTarea for grasp " << line << " is: " << QTarea << endl;
    //visualize(graspPointCloud, partialObjectPC, CM, cloudIn, min, max, rotation, translation, false);
    return true;
}

bool MVBB::loadPointCloud(string path, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *pointCloud) == -1) {
        PCL_ERROR ("Can't read file .pcd \n");
        return false;
    }
}

bool MVBB::read_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &C_Object, 
                       pcl::PointCloud<pcl::Normal>::Ptr &normals) 
                       {
    std::fstream obj;
    std::vector<std::vector <double>> points;
    std::vector<std::vector <double>> norms;
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
            for(int i = 0; i < 6; i ++) {
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

void MVBB::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered) {
    
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

void MVBB::getGraspQuality(const char *absPath1, 
                           const char *absPath3) { 
    //get the quality of each grasp from the .xml file and save it into a .txt file
    ofstream GraspQuality(absPath3);
    double quality;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(absPath1);
    if (!result) std::cout << "Parse error: " << result.description() << ", character pos = " << result.offset<<std::endl;
    else std::cout << "Problem file loaded"<<std::endl;
    int i = 0;
    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")) {
        quality = tool.attribute("quality").as_double();
        GraspQuality << quality << endl;
        std::cout << quality << std::endl;
        i++;
    }
}

void MVBB::getTransforms(const char *absPath1, 
                         const char *absPath2) {   
    //get the matrix transformation of each grasp from the .xml file and save them into a .txt file
    ofstream transform(absPath2);
    double xx, xy, xz, yx, yy, yz, zx, zy, zz, x1, y1, z1, p1, p2, p3, p4;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(absPath1);
    if (!result) std::cout << "Parse error: " << result.description() << ", character pos = " << result.offset<<std::endl;
    else std::cout << "Problem file loaded"<<std::endl;
    int i = 0;
    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")){

        pugi::xml_node node = tool.child("Transform").child("Matrix4x4").child("row1");
        xx = node.attribute("c1").as_double();
        xy = node.attribute("c2").as_double();
        xz = node.attribute("c3").as_double();
        x1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row2");

        yx = node.attribute("c1").as_double();
        yy = node.attribute("c2").as_double();
        yz = node.attribute("c3").as_double();
        y1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row3");

        zx = node.attribute("c1").as_double();
        zy = node.attribute("c2").as_double();
        zz = node.attribute("c3").as_double();
        z1 = node.attribute("c4").as_double();

        node = tool.child("Transform").child("Matrix4x4").child("row4");

        p1 = node.attribute("c1").as_double();
        p2 = node.attribute("c2").as_double();
        p3 = node.attribute("c3").as_double();
        p4 = node.attribute("c4").as_double();

        transform << xx << ", ";
        transform << xy << ", ";
        transform << xz << ", ";
        transform << x1 << ", ";

        transform << yx << ", ";
        transform << yy << ", ";
        transform << yz << ", ";
        transform << y1 << ", ";

        transform << zx << ", ";
        transform << zy << ", ";
        transform << zz << ", ";
        transform << z1 << ", ";

        transform << p1 << ", ";
        transform << p2 << ", ";
        transform << p3 << ", ";
        transform << p4 << ";" << endl;

        //cout << "Transform (" << i << ") " << xx << " " << xy << " " << xz << " " << x1 << " " << yx << " " << yy << " " << yz << " " << y1 << " " << zx << " " << zy << " " << zz << " " << z1
        //     << " " << p1 << " " << p2 << " " << p3 << " " << p4 << endl;
        i++;
    }
}

void MVBB::QualitySort(const char *absPath4, 
                       const char *absPath5) {
                        
    ofstream GraspQuality(absPath5);
    double quality;
    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(absPath4);
    if (!result){
        std::cout << "Parse error: " << result.description()
                  << ", character pos = " << result.offset<<std::endl;
    }
    else{
        std::cout << "Problem file loaded"<<std::endl;
    }

    for(pugi::xml_node tool = doc.child("ManipulationObject").child("GraspSet").child("Grasp"); tool; tool = tool.next_sibling("Grasp")){

        int i = 0;
        quality = tool.attribute("quality").as_double();
        GraspQuality << quality << endl;
        i++;
    }

    // Open your text file
    std::ifstream file("/home/fernando/PHD/Experiments/grasps/Coffee_mug/SortQualities.txt");
    std::vector<std::string> rows;

    // Read all the lines and add them to the rows vector
    while(!file.eof())
    {
        std::string line;
        std::getline(file, line);
        rows.push_back(line);
    }

    // Sort the vector
    std::sort(rows.begin(), rows.end());

    // Print out all of the vectors values
    std::vector<std::string>::iterator iterator = rows.begin();
    for(; iterator != rows.end(); ++iterator)
        //std::cout << *iterator << std::endl;
        getchar();
}

int MVBB::extractGraspNumber(string graspPointCloudPath) {
    // For atoi, the input string has to start with a digit, so lets search for the first digit
    size_t i = 0;
    for ( ; i < graspPointCloudPath.length(); i++ ) { 
        if (isdigit(graspPointCloudPath[i])) 
            break; 
        }
    // remove the first chars, which aren't digits
    graspPointCloudPath = graspPointCloudPath.substr(i, graspPointCloudPath.length() - i );
    int number = atoi(graspPointCloudPath.c_str());
    return number;
}

Eigen::Matrix4f MVBB::returnTransformation(string transformationFilePath, 
                                           uint line) {
    string sLine = "";
    ifstream read;
    read.open(transformationFilePath);
    int line_no = 0;
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
    transformation << num[0], 
                      num[1], 
                      num[2], 
                      num[3], 
                      num[4], 
                      num[5], 
                      num[6], 
                      num[7], 
                      num[8], 
                      num[9], 
                      num[10], 
                      num[11], 
                      num[12], 
                      num[13], 
                      num[14], 
                      num[15];
    read.close();
    return transformation;
}

void MVBB::getHandPCTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &Hand_configuration, 
                                   Eigen::Quaternionf &BBox_Rotation,
                                   Eigen::Vector3f &BBox_Translation, 
                                   Eigen::Vector4f &min, 
                                   Eigen::Vector4f &max, 
                                   Eigen::Matrix4f &projection,
                                   Eigen::Matrix4f transform) {

    Eigen::Matrix4f Tinv;
    Tinv = transform.inverse();

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_grasp (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*Hand_configuration, *transformed_cloud_grasp, Tinv);

    ///GRASP MVBB
    //Compute PCA
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*transformed_cloud_grasp, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*transformed_cloud_grasp, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    //Transform the original grasp point cloud to the origin where the principal component correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*transformed_cloud_grasp, *cloudPointsProjected, projectionTransform);
    projection = projectionTransform;

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    BBox_Rotation = bboxQuaternion;
    BBox_Translation = bboxTransform;
    min[0] = minPoint.x;
    min[1] = minPoint.y;
    min[2] = minPoint.z;
    max[0] = maxPoint.x;
    max[1] = maxPoint.y;
    max[2] = maxPoint.z;
    Hand_configuration->clear();
    Hand_configuration = transformed_cloud_grasp;
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
    gp.setMaximumNearestNeighbors(100);
    gp.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp.setMinimumAngle(M_PI / 18); // 10 degrees
    gp.setMaximumAngle(2 * (M_PI / 3)); // 120 degrees
    gp.setNormalConsistency(false);

    // Get result
    gp.setInputCloud(C_ObjectNormals);
    gp.setSearchMethod(tree2);
    gp.reconstruct(triangles);

    //calculate area
    pcl::PointCloud<pcl::PointXYZ> cloud_area;
    cloud_area = *C_Object;

    int index_p1, index_p2, index_p3;
    double x1, x2, x3, y1, y2, y3, z1, z2, z3, a, b, c, q;
    float area = 0.0;

    for(int i = 0; i < triangles.polygons.size(); i++) {
        index_p1 = triangles.polygons[i].vertices[0];
        index_p2 = triangles.polygons[i].vertices[1];
        index_p3 = triangles.polygons[i].vertices[2];

        x1 = cloud_area.points[index_p1].x;
        y1 = cloud_area.points[index_p1].y;
        z1 = cloud_area.points[index_p1].z;

        x2 = cloud_area.points[index_p2].x;
        y2 = cloud_area.points[index_p2].y;
        z2 = cloud_area.points[index_p2].z;

        x3 = cloud_area.points[index_p3].x;
        y3 = cloud_area.points[index_p3].y;
        z3 = cloud_area.points[index_p3].z;

        //Heron's formula:
        a = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2) + pow((z1 - z2),2));
        b = sqrt(pow((x1 - x3),2) + pow((y1 - y3),2) + pow((z1 - z3),2));
        c = sqrt(pow((x3 - x2),2) + pow((y3 - y2),2) + pow((z3 - z2),2));
        q = (a + b + c) / 2;

        area = area + sqrt(q * (q - a) * (q - b) * (q - c));
    }
    return area;
}

void MVBB::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr Hand_configuration, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr Points_out,
                     Eigen::Vector3f Ctr, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr Points_in, 
                     Eigen::Vector4f Min,
                     Eigen::Vector4f Max, 
                     Eigen::Quaternionf BBox_Rotation, 
                     Eigen::Vector3f BBox_Translation, 
                     bool f_coordinates) {

    pcl::visualization::PCLVisualizer BBox_Visualizer ("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filteredColorOut(Points_out, 0, 255, 0); //Points out the box (green)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filteredColorIn(Points_in, 0, 0, 255);  //Points in the box (blue)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> grasp_cloud (Hand_configuration, 255, 0, 0); //Grasp (red)
    BBox_Visualizer.setBackgroundColor(255,255,255); // white background
    BBox_Visualizer.addPointCloud(Points_out, filteredColorOut, "cloud_out");
    BBox_Visualizer.addPointCloud(Points_in, filteredColorIn, "cloud_in");
    BBox_Visualizer.addPointCloud(Hand_configuration, grasp_cloud, "grasp_cloud");
    BBox_Visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "cloud_out");
    BBox_Visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "cloud_in");
    BBox_Visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.2, "grasp_cloud");
    if(f_coordinates) {
        BBox_Visualizer.addCoordinateSystem(10,"world",0);
        BBox_Visualizer.addCoordinateSystem(10,Ctr[0],Ctr[1],Ctr[2],"centroid",0);
    }
    BBox_Visualizer.addCube(BBox_Translation, BBox_Rotation, Max[0] - Min[0], Max[1] - Min[1], Max[2] - Min[2], "boundingbox", 0);
    //BBox_Visualizer.setRepresentationToWireframeForAllActors();
    BBox_Visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "boundingbox");
    BBox_Visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "boundingbox");
    BBox_Visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "boundingbox");

    while(!BBox_Visualizer.wasStopped()) {
        BBox_Visualizer.spinOnce();
    }
    BBox_Visualizer.close();
}