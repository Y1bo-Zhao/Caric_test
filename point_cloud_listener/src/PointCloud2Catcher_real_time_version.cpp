#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <fstream>

#include <vector>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

// 创建一个新的点云对象，用于存储所有的点
pcl::PointCloud<pcl::PointXYZ>::Ptr all_clouds(new pcl::PointCloud<pcl::PointXYZ>);
// 如果已经搞过一遍bbox了，就不搞了
bool finish_bbox_record_R = false;

class PointCloudObject
{
private:
    pcl::PointXYZ point; // 点云坐标
    int boundingBoxId;  // 所属的bounding box的id
    pcl::Normal normal; // 点云对应的法向量

public:
    // 构造函数
    PointCloudObject(const pcl::PointXYZ& point, int boundingBoxId, const pcl::Normal& normal)
        : point(point), boundingBoxId(boundingBoxId), normal(normal) {}

    // 获取点云坐标
    pcl::PointXYZ getPoint() const { return point; }

    // 获取所属的bounding box的id
    int getBoundingBoxId() const { return boundingBoxId; }

    // 获取点云对应的法向量
    pcl::Normal getNormal() const { return normal; }
};



class Boundingbox
{
public:

    Boundingbox()
    {
        center = Eigen::Vector3d(0, 0, 0);
        volume = 0;
        id = -1;
    };


    double getXsize() const { return xsize; }
    double getYsize() const { return ysize; }
    double getZsize() const { return zsize; }

    Boundingbox(string str)
    {
        vector<string> spilited_str;
        std::istringstream iss(str);
        std::string substring;
        while (std::getline(iss, substring, ','))
        {
            spilited_str.push_back(substring);
        }
        
        int i = 0;
        while (i < 24)
        {
            vertice.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        while (i < 33)
        {
            rotation_matrix(i - 24) = stod(spilited_str[i]);
            i++;
        }
        while (i < 36)
        {
            center = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 39)
        {
            size_vector = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 42)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        while (i < 45)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        xsize = stod(spilited_str[i]);
        i++;
        ysize = stod(spilited_str[i]);
        i++;
        zsize = stod(spilited_str[i]);
        i++;
        id = stod(spilited_str[i]);
        i++;
        state = stod(spilited_str[i]);
        i++;
        volume = stod(spilited_str[i]);
        i++;
        use_x = stod(spilited_str[i]);
        i++;
        use_y = stod(spilited_str[i]);
        i++;
        use_z = stod(spilited_str[i]);
        i++;
    }//这个类定义了一个边界框，并提供了两个构造函数。第一个构造函数是默认的，
     //它初始化了一个中心在(0, 0, 0)的边界框，并为volume和id赋了初值。
     //第二个构造函数接受一个字符串参数，并从该字符串中解析出边界框的属性值，如顶点、旋转矩阵、中心、大小向量、全局输入/输出等。

    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in, Eigen::Vector3d &start, Eigen::Vector3d &end, int state_in, bool x, bool y, bool z)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;//算中心
        xsize = (vec[1] - vec[0]).norm();//求x,y,z向量的长度
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();
        volume = xsize * ysize * zsize;//计算体积
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);//计算大小向量
        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();
        rotation_matrix << xaxis, yaxis, zaxis;//计算旋转矩阵
        global_in_out.push_back(start);
        global_in_out.push_back(end);
        state = state_in;
        use_x = x;
        use_y = y;
        use_z = z;
    }
    
    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();

        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);

        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();

        Eigen::Vector3d xplus, xminus, yplus, yminus, zplus, zminus;//这里声明了六个3D向量。这六个向量分别代表边界框在X、Y、Z三个轴上的正方向（plus）和负方向（minus）面的中心。
        yminus = (vec[0] + vec[1] + vec[4] + vec[5]) / 4;//这里计算了边界框在Y轴的两个面的中心点。yminus是Y轴负方向面的中心，而yplus是Y轴正方向面的中心。
        yplus = (vec[2] + vec[3] + vec[6] + vec[7]) / 4;

        xminus = (vec[0] + vec[3] + vec[4] + vec[7]) / 4;
        xplus = (vec[1] + vec[2] + vec[5] + vec[6]) / 4;

        zminus = (vec[0] + vec[1] + vec[2] + vec[3]) / 4;
        zplus = (vec[4] + vec[5] + vec[6] + vec[7]) / 4;

        rotation_matrix << xaxis, yaxis, zaxis;//计算旋转矩阵
        if ((zsize >= ysize) && (zsize >= xsize))//以下代码选择了Boundingbox最大的面，并将该面的中心作为全局输入/输出点。
        {
            use_z = true;
            global_in_out.push_back(zminus);
            global_in_out.push_back(zplus);
        }
        else if ((xsize >= ysize) && (xsize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_x = true;
                global_in_out.push_back(xminus);
                global_in_out.push_back(xplus);
            }
        }
        else if ((ysize >= xsize) && (ysize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_y = true;
                global_in_out.push_back(yminus);
                global_in_out.push_back(yplus);
            }
        }
        else
        {
            if (global_in_out.size() == 0)
            {
                use_z = true;
                global_in_out.push_back(zminus);
                global_in_out.push_back(zplus);
            }
        }
    };

    ~Boundingbox(){};
    
    const Matrix3d getSearchRotation() const
    {
        Eigen::Vector3d axis_rotation_along(0.0, 0.0, 1.0);
        Eigen::Matrix3d transfer_matrix;
        Eigen::Matrix3d result;
        double angle = 0;
        if (use_x)
        {
            if (state == 0)
            {
                cout << "x+" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = -M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "x-" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                cout << "y+" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "y-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = -M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                cout << "z+" << endl;
            }
            else if (state == 1)
            {
                cout << "z-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }

        transfer_matrix = Eigen::AngleAxisd(angle, axis_rotation_along);
        result = transfer_matrix * rotation_matrix.inverse();
        return result;
        
    }

    int getState()
    {
        return state;
    }

    double getVolume() const
    {
        return volume;
    }
    const Vector3d getCenter() const
    {
        return center;
    }
    const Matrix3d getRotation() const
    {
        return rotation_matrix;
    }
    const Vector3d getExtents() const
    {
        return size_vector;
    }
    const Vector3d getRotExtents() const
    {
        Eigen::Vector3d result(0, 0, 0);
        if (use_x)
        {
            if (state == 0)
            {
                // cout<<"x+"<<endl;
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
            }
            else if (state == 1)
            {
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
                // cout<<"x-"<<endl;
                // axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                // angle=M_PI/2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                // cout<<"y+"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else if (state == 1)
            {
                // cout<<"y-"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                // cout<<"z+"<<endl;
                result = size_vector;
            }
            else if (state == 1)
            {
                // cout<<"z-"<<endl;
                result = size_vector;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "nothing happen" << endl;
        }
        return result;
    }
    vector<Eigen::Vector3d> getVertices() const
    {
        return vertice;
    }
    Eigen::Vector3d get_global_in_out(int state) const
    {
        Eigen::Vector3d result = global_in_out[state];
        return result;
    }
    void edit_state(int state_in)
    {
        state = state_in;
    }
    void edit_id()
    {
        id = id + 1;
    }
    int getId()
    {
        return id;
    }

    void generate_start(double scale, Boundingbox &start, Boundingbox &end)
    {
        if (use_z)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else if (use_x)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
    }
//这个函数的基本思路是根据当前边界框、选择的轴以及给定的缩放因子来计算两个新的边界框。
//这些新的边界框可能用于模拟某种物体或结构在某个方向上的运动或形变。

    string generate_string_version() const
    {
        string result = "";
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(vertice[i][j]) + ",";
            }
        }
        for (int i = 0; i < 9; i++)
        {
            result = result + to_string(rotation_matrix(i)) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(center[j]) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(size_vector[j]) + ",";
        }

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(global_in_out[i][j]) + ",";
            }
        }
        result = result + to_string(xsize) + ",";
        result = result + to_string(ysize) + ",";
        result = result + to_string(zsize) + ",";
        result = result + to_string(id) + ",";
        result = result + to_string(state) + ",";
        result = result + to_string(volume) + ",";
        result = result + to_string(use_x) + ",";
        result = result + to_string(use_y) + ",";
        result = result + to_string(use_z) + ",";
        // cout<<result<<endl;
        return result;
    }//这里再把得出来的结果输出回去。
    
private:
    vector<Eigen::Vector3d> vertice; 
    double volume = 0;
    Eigen::Vector3d center;                
    Eigen::Matrix3d rotation_matrix;       
    Eigen::Vector3d size_vector;           
    double xsize, ysize, zsize;            
    int id = 0;                            
    vector<Eigen::Vector3d> global_in_out; 
    int state = 0;                         
    bool use_x = false;                    
    bool use_y = false;                    
    bool use_z = false;                    
};










// 定义节点级别的bounding box作为全局向量
vector<Boundingbox> boxes;

// 输出数据集
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

// 来个容器
std::map<int, std::vector<PointCloudObject>> objectsMap;














void MatchPointCloudWithBbox()
{   
    objectsMap.clear();
    assert(all_clouds->points.size() == cloud_normals->points.size());

    // 遍历点云中的所有点
    for (size_t i = 0; i < all_clouds->points.size(); ++i)
    {
        // 获取点和对应的法线向量
        const auto& point = all_clouds->points[i];
        const auto& normal = cloud_normals->points[i];

        // 检查点是否在bounding box内
        for(int j=0; j<boxes.size(); j++)
        {
            if (point.x >= boxes[j].getCenter().x() - 0.5 * boxes[j].getXsize() && point.x <= boxes[j].getCenter().x() + 0.5 * boxes[j].getXsize() &&
                point.y >= boxes[j].getCenter().y() - 0.5 * boxes[j].getYsize() && point.y <= boxes[j].getCenter().y() + 0.5 * boxes[j].getYsize() &&
                point.z >= boxes[j].getCenter().z() - 0.5 * boxes[j].getZsize() && point.z <= boxes[j].getCenter().z() + 0.5 * boxes[j].getZsize())
            {
                // 这个点在bounding box内
                objectsMap[j].push_back(PointCloudObject(point, j, normal));
            }
        }
    }
}







//法向量计算
void NormalVectorCompute()
{
    // 创建一个NormalEstimation对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(all_clouds);

    // 创建一个空的kdtree对象，并把它传递给法向量估计对象
    // 它的内容将根据给定的输入数据集填充到对象内部（因为并没有预先计算）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);


    // 使用k近邻搜索，k设为10
    ne.setKSearch(20);

    // 计算特征值
    ne.compute(*cloud_normals);
}




// 回调函数，当接收到PointCloud2消息时会被调用
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 将PointCloud2消息转换为PCL的PointCloud对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // 创建一个条件滤波器range_cond
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
    // 筛掉了地面的点云
    // 使用条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);

    // 执行滤波操作
    condrem.filter (*cloud);

    // 使用removeNaNFromPointCloud函数来去除NaN点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    
    // 改稀疏一点,创建一个VoxelGrid滤波器
    // 执行滤波操作
    // 将新的点云数据添加到all_clouds中
    *all_clouds += *cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (all_clouds);
    sor.setLeafSize (2.0f, 2.0f, 2.0f);
    sor.filter(*all_clouds);

    //实时计算每个点的法向量
    NormalVectorCompute();

    
}





//和task assign里面的方程一样，只不过把box_set改成了boxes，然后把finish...后面加了_R避免重合
void verticesCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    if(finish_bbox_record_R == true)
    {
        // cout<<"now agent get"<<endl;
        // for(auto& name:namelist)
        // {
        //     if(position_pair[name].update){
        //         cout<<name<<endl;
        //     }
        // }
        return;
    }
    sensor_msgs::PointCloud cloud = *msg;
    int num_points = cloud.points.size();
    if (num_points % 8 == 0 && num_points > 8 * boxes.size())
    {
        int num_box = num_points / 8;
        for (int i = 0; i < num_box; i++)
        {
            double xmax = -std::numeric_limits<double>::max();
            double ymax = -std::numeric_limits<double>::max();
            double zmax = -std::numeric_limits<double>::max();
            double xmin = std::numeric_limits<double>::max();
            double ymin = std::numeric_limits<double>::max();
            double zmin = std::numeric_limits<double>::max();
            vector<Eigen::Vector3d> point_vec;
            for (int j = 0; j < 8; j++)
            {
                if (cloud.points[8 * i + j].x > xmax)
                {
                    xmax = cloud.points[8 * i + j].x;
                }
                if (cloud.points[8 * i + j].x < xmin)
                {
                    xmin = cloud.points[8 * i + j].x;
                }
                if (cloud.points[8 * i + j].y > ymax)
                {
                    ymax = cloud.points[8 * i + j].y;
                }
                if (cloud.points[8 * i + j].y < ymin)
                {
                    ymin = cloud.points[8 * i + j].y;
                }
                if (cloud.points[8 * i + j].z > zmax)
                {
                    zmax = cloud.points[8 * i + j].z;
                }
                if (cloud.points[8 * i + j].z < zmin)
                {
                    zmin = cloud.points[8 * i + j].z;
                }
                point_vec.push_back(Eigen::Vector3d(cloud.points[8 * i + j].x, cloud.points[8 * i + j].y, cloud.points[8 * i + j].z));
            }
            boxes.push_back(Boundingbox(point_vec, i));
            point_vec.clear();
        }
        finish_bbox_record_R = true;
        //Team allocate
       // Team_allocate();
        //Use best first to generate the global trajactory
       // Best_first_search();
        //Clip the best path
       // Clip_the_task();
        //Generate the massage
       // generate_massage();


    }
    else
    {
        return;
    }
    return;
}




 //这个代码用来检查objectMap是不是对的。
void SaveObjectsMapToFile()
{
    // 创建一个输出文件流对象
    std::ofstream outFile("ObjectsMapOutput.txt");

    // 检查文件是否成功打开
    if (!outFile)
    {
        std::cerr << "无法打开文件" << std::endl;
        return;
    }

    // 遍历objectsMap
    for (const auto& pair : objectsMap)
    {
        // 将键（key）写入到文件中
        outFile << "Key: " << pair.first << "\n";

        // 遍历与当前键关联的PointCloudObject列表
        for (const auto& object : pair.second)
        {
            // 将PointCloudObject的信息写入到文件中
            outFile << "PointCloudObject: " 
                    << "Point: (" << object.getPoint().x << ", " << object.getPoint().y << ", " << object.getPoint().z << "), "
                    << "BoundingBoxId: " << object.getBoundingBoxId() << ", "
                    << "Normal: (" << object.getNormal().normal_x << ", " << object.getNormal().normal_y << ", " << object.getNormal().normal_z << ")\n";
        }

        // 在每个键值对之间添加一个空行，以便于阅读
        outFile << "\n";
    }

    // 关闭文件
    outFile.close();
}






/* //这个函数用来发布点云消息，我想试试看能不能再rviz里面画出来点云的点
void PointCloudPub(ros::Publisher pcl_pub)
{
    // 将点云数据转换为ROS消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*all_clouds, output);
    output.header.frame_id = "map";

    pcl_pub.publish(output);

}

//这个函数用来发布法向量消息，我再看看
void NormalVectorPub(ros::Publisher norm_pub)
{
    // 将点云数据转换为ROS消息
    geometry_msgs::Vector3Stamped output;
    pcl::toROSMsg(*all_clouds, output);
    output.header.frame_id = "map";

    pcl_pub.publish(output);

} */


//遍历ObjectMap，输出向量看看怎么个事儿
void VectorOutput(ros::Publisher& marker_pub, visualization_msgs::MarkerArray& marker_array)
{
    int id = 0;//这个是给marker分配的id
    for (const auto& pair : objectsMap)
        {   
            for (const auto& object : pair.second)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time();
                marker.ns = "ILoveYou";
                marker.id = id;
                id++;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                // 设置箭头的起点和终点
                geometry_msgs::Point start, end;
                start.x = object.getPoint().x ; 
                start.y = object.getPoint().y ; 
                start.z = object.getPoint().z ;
                end.x = object.getPoint().x + object.getNormal().normal_x ; 
                end.y = object.getPoint().y + object.getNormal().normal_y ; 
                end.z = object.getPoint().z + object.getNormal().normal_z ;
                marker.points.push_back(start);
                marker.points.push_back(end);


                // 设置箭头的大小
                marker.scale.x = 0.1;  // 箭头的宽度
                marker.scale.y = 0.2;  // 箭头的高度
                marker.scale.z = 0.3;  // 箭头的长度

                // 设置箭头的颜色
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                
                // 将Marker添加到MarkerArray中
                marker_array.markers.push_back(marker);
            }
        }
}


// 创建一个MarkerArray消息
visualization_msgs::MarkerArray marker_array;


int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "jurong_PC_listener");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个订阅者，订阅名为"jurong/cloud_inW"的topic，注册回调函数cloudCallback
    ros::Subscriber LocalPointCloudSub = nh.subscribe("/jurong/cloud_inW", 1, cloudCallback);
    // 创建一个订阅者，订阅名为"/gcs/bounding_box_vertices"的topic，注册回调函数Callback
    ros::Subscriber BboxVerticesSub = nh.subscribe("/gcs/bounding_box_vertices", 1, verticesCallback);

/*     // 创建一个Publisher发布点云消息
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); */
    // 创建一个Publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("normal_marker_array", 1);

    // 设置发布频率为10Hz
    ros::Rate loop_rate(0.1);


    // 进入循环，等待回调函数被调用
    while (ros::ok()) {
        // PointCloudPub(pcl_pub);
        // 建立对象ROS
        MatchPointCloudWithBbox();
        VectorOutput(marker_pub, marker_array);
        marker_pub.publish(marker_array);//发布向量消息
        ros::spinOnce();
        // 按照设定的频率休眠
        loop_rate.sleep();
    }

    // 保存所有的点云数据到一个文件中
    pcl::io::savePCDFileASCII("all_jurong_pcd.pcd", *all_clouds);
    
    SaveObjectsMapToFile();
    std::cout << "Saved " << all_clouds->points.size() << " data points to all_jurong_pcd.pcd." << std::endl;
    std::cout << "Saved Object Map to File 'ObjectsMapOutput.txt'" << std::endl;

    return 0;
}

