#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/geometry/distance.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>


#include "Astar.h"
#include "Axing.h"
#include <map>

#include "std_msgs/Int32.h"

#include <std_msgs/Bool.h>

//智能指针，用在全局的点云里：
pcl::PointCloud<pcl::PointXYZ>::Ptr all_clouds(new pcl::PointCloud<pcl::PointXYZ>);

class Boundingbox
{
public:

    Boundingbox()
    {
        center = Eigen::Vector3d(0, 0, 0);
        volume = 0;
        id = -1;
    };

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
    }

    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in, Eigen::Vector3d &start, Eigen::Vector3d &end, int state_in, bool x, bool y, bool z)
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
        rotation_matrix << xaxis, yaxis, zaxis;
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

        Eigen::Vector3d xplus, xminus, yplus, yminus, zplus, zminus;
        yminus = (vec[0] + vec[1] + vec[4] + vec[5]) / 4;
        yplus = (vec[2] + vec[3] + vec[6] + vec[7]) / 4;

        xminus = (vec[0] + vec[3] + vec[4] + vec[7]) / 4;
        xplus = (vec[1] + vec[2] + vec[5] + vec[6]) / 4;

        zminus = (vec[0] + vec[1] + vec[2] + vec[3]) / 4;
        zplus = (vec[4] + vec[5] + vec[6] + vec[7]) / 4;

        rotation_matrix << xaxis, yaxis, zaxis;
        if ((zsize >= ysize) && (zsize >= xsize))
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
            cout << "noting happen" << endl;
        }
        return result;
    }
    vector<Eigen::Vector3d> getVertices() const
    {
        return vertice;
    }
    Eigen::Vector3d get_global_in_out(int state) const
    {
        Eigen::Vector3d result = global_in_out[state]; //state=0用负面，state=1用正面
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
    }
    
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



class Agent
{
public:
    Agent(ros::NodeHandlePtr &nh_ptr_)
    : nh_ptr(nh_ptr_)
    {
        //把该explorer需要的下一个位置通过/task_assign传进来，在回调中直接用cmd_pub_0、cmd_pub_1发送控制指令
        task_sub_ = nh_ptr->subscribe("/task_assign" + nh_ptr->getNamespace(), 10, &Agent::TaskCallback, this);
        
        //-------------------------------------------------------------------------------------------------
        //处理/broadcast的LOS问题
        client    = nh_ptr->serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");
        communication_pub_ = nh_ptr->advertise<std_msgs::String>("/broadcast", 10);

        string str = nh_ptr->getNamespace();
        str.erase(0, 1);
        srv.request.source = str;
        srv.request.targets.push_back("all");
        srv.request.topic_name = "/broadcast";
        srv.request.package_name = "std_msgs";
        srv.request.message_type = "String";
        while (!serviceAvailable)
        {
            serviceAvailable = ros::service::waitForService("/create_ppcom_topic", ros::Duration(10.0));
        }
        string result = "Begin";
        while (result != "success lah!")
        {
            client.call(srv);
            result = srv.response.result;
            printf(KYEL "%s\n" RESET, result.c_str());
            std::this_thread::sleep_for(chrono::milliseconds(1000));//c++进程休眠1秒
        }
        communication_initialise = true;
        cout << "communication_initialise"<<endl;
        //实现LOS，通过订阅/broadcast/Namespace代替订阅/broadcast
        //-------------------------------------------------------------------------------------------------

        //拿到位置后用communication_pub_传到gcs里用以初始定位并分配box顺序。
        

        cmd_pub = nh_ptr->advertise<trajectory_msgs::MultiDOFJointTrajectory>(nh_ptr->getNamespace() + "/command/trajectory",1);
        Los_sub_ = nh_ptr->subscribe("/LOS",10, &Agent::TopologyCallback, this);
        box_sub_ = nh_ptr->subscribe("/box",10, &Agent::BoxCallback, this);
        box_index_sub_ = nh_ptr->subscribe("/start",10, &Agent::BoxindexCallback, this);

        odom_sub_        = nh_ptr->subscribe("/ground_truth/odometry", 10, &Agent::OdomCallback, this);//map

        //------------gimabl
        gimbal_pub_ = nh_ptr->advertise<geometry_msgs::Twist>("/firefly/command/gimbal", 1);
        //TimerCmdOut   = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &Agent::TimerCmdOutCB,   this);
        Pos_sub_   = nh_ptr->subscribe("/ground_truth/odometry", 10, &Agent::PosCallback, this);
        

        //Massage_subscribe_Timer=nh_ptr_->createTimer(ros::Duration(1.0/10.0),  &Agent::cmd_publisher,     this);

        pointcloud_sub_ = nh_ptr->subscribe("/cloud_inW", 10, &Agent::pcCallback, this);

        back_pub = nh_ptr->advertise<std_msgs::Int32>(nh_ptr->getNamespace() + "/back",1);
    }

private:
    ros::NodeHandlePtr nh_ptr;  // nodehandle for communication\\

    ros::Publisher cmd_pub;//发送控制指令
    ros::Publisher back_pub; //发送绕盒情况

    /*ros::Timer TimerProbeNbr;   // To request updates from neighbours
    ros::Timer TimerPlan;       // To design a trajectory
    ros::Timer TimerCmdOut;     // To issue control setpoint to unicon
    ros::Timer TimerViz;        // To vizualize internal states

    */
    ros::Subscriber Pos_sub_;

    // part 1
    caric_mission::CreatePPComTopic srv; // This PPcom create for communication between neibors;
    ros::ServiceClient client;           // The client to create ppcom
    ros::Publisher communication_pub_;   // PPcom publish com
    bool serviceAvailable = false;       // The flag whether the communication service is ready
    ros::Subscriber task_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber Los_sub_;
    bool message;
    int index = 1;

    std_msgs::String msg_1;
    ros::Timer Massage_subscribe_Timer;

    
    ros::Subscriber box_sub_;
    ros::Subscriber box_index_sub_;

    //----避障------
    ros::Subscriber pointcloud_sub_;



    //--------------
    

    Eigen::Vector3d start_point;
    bool getstart = false;
    bool get_5points = false;

    int target_box_0=0;
    Eigen::Vector3d target_point_0;
    int point_index_0=0;
    vector<Eigen::Vector3d> point_0;
    int height_index_0=0;
    Eigen::Vector3d point_temp_0;
    
    bool inout = false;
    Eigen::Vector3d initial_position;

    //-----------；gimbal------
    //ros::Timer TimerCmdOut;
    ros::Publisher gimbal_pub_;
    geometry_msgs::Twist gimbal_msg;

    //--------------避障---------------
    Boundingbox Box;
    Eigen::Vector3d start;
    Eigen::Vector3d goal;
    vector<Obstacle> obstacles;
    bool get_obstacles = false;
    
    Eigen::Vector3d now_position;
    int path_index=1;
    Eigen::Vector3d path_point;
    vector<Eigen::Vector3d> path;
    


    
    //ros::Subscriber com_sub_;
    //string pre_task;

    /*
    // callback Q2
    ros::Subscriber odom_sub_;   // Get neibor_info update
    ros::Subscriber gimbal_sub_; // Get gimbal info update;
    // callback Q3
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *nbr_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>       *odom_filter_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry> MySyncPolicy;
    // // boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;

    // callback Q4
    ros::Publisher motion_pub_; // motion command pub
    ros::Publisher gimbal_pub_; // motion gimbal pub

    // callback Q5
    ros::Publisher map_marker_pub_;
    ros::Publisher path_pub_;

    mainbrain mm;

    // variable for static map
    vector<Eigen::Vector3d> Nbr_point;
    */

    bool communication_initialise = false;
    bool go = false;
    
    bool inout_1 = false;
    bool inout_flag = false;
    bool get_now_position = false;
    // Callback function

    //输入需要xyz、偏航角
    void TaskCallback(const std_msgs::String msg)
    {
        msg_1 = msg;
        //cout << "收到" <<endl;
        go = true;
         
    }

    void cmd_publisher(const ros::TimerEvent &)
    {
        Eigen::Vector3d target_pos(0,0,0);
        

        vector<string> spilited_str;
        //spilited_str.resize(2);
        std::istringstream iss(msg_1.data);
        std::string substring;
        while (std::getline(iss, substring, ';'))
        {
            spilited_str.push_back(substring);
        }
        
        

        if(nh_ptr->getNamespace() == "/jurong")
        {
            target_pos=str2point(spilited_str[0]);
            //cout << "jurong收到"<< spilited_str[0]<<endl;

        }
        if(nh_ptr->getNamespace() == "/raffles")
        {
            target_pos=str2point(spilited_str[1]);
            //cout << "raffles收到"<< spilited_str[1]<<endl;
        }
        cout << "收到的目标"<<target_pos <<endl;
        double target_yaw=0;

        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

        geometry_msgs::Transform transform_msg;//Vector3 translation、Quaternion rotation
        geometry_msgs::Twist accel_msg, vel_msg;

        transform_msg.translation.x = target_pos(0);
        transform_msg.translation.y = target_pos(1);
        transform_msg.translation.z = target_pos(2);
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(target_yaw*0.5);
        transform_msg.rotation.w = cosf(target_yaw*0.5);

        trajpt_msg.transforms.push_back(transform_msg);

        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        accel_msg.linear.x = 0;
        accel_msg.linear.x = 0;
        accel_msg.linear.x = 0;

        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajset_msg.header.stamp = ros::Time::now();
        cmd_pub.publish(trajset_msg);

    }

    void PosCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        now_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        get_now_position = true;
    }

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg) //把里程计的数据pub到broadcast（与gcs的positionCallback相关）
    {
        initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        std_msgs::String init_position_msg;
        init_position_msg.data="init_pos;"+nh_ptr->getNamespace()+";"+to_string(initial_position.x())+","+to_string(initial_position.y())+","+to_string(initial_position.z());
        if(communication_initialise)
        {
            communication_pub_.publish(init_position_msg);
        }
        
        if(!get_obstacles || !get_now_position)
        {
            return;
        }
        Get_next_point_to_go();
        //cout << "现在的位置：" << now_position <<endl;
        //cout << "目标点：" << path_point <<endl;
        
        if(go==true&&getstart)
        {
            Eigen::Vector3d target_pos(0,0,0);
            target_pos = path_point;
            double target_yaw=0;

            trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

            geometry_msgs::Transform transform_msg;//Vector3 translation、Quaternion rotation
            geometry_msgs::Twist accel_msg, vel_msg;

            transform_msg.translation.x = 0;
            transform_msg.translation.y = 0;
            transform_msg.translation.z = 0;
            transform_msg.rotation.x = 0;
            transform_msg.rotation.y = 0;
            transform_msg.rotation.z = sinf(target_yaw*0.5);
            transform_msg.rotation.w = cosf(target_yaw*0.5);

            trajpt_msg.transforms.push_back(transform_msg);
            

            vel_msg.linear.x = (target_pos(0)-initial_position.x())/((target_pos-initial_position).norm())*2;
            vel_msg.linear.y = (target_pos(1)-initial_position.y())/((target_pos-initial_position).norm())*2;
            vel_msg.linear.z = (target_pos(2)-initial_position.z())/((target_pos-initial_position).norm())*2;
            

            accel_msg.linear.x = 0;
            accel_msg.linear.y = 0;
            accel_msg.linear.z = 0;

            trajpt_msg.velocities.push_back(vel_msg);
            trajpt_msg.accelerations.push_back(accel_msg);
            trajset_msg.points.push_back(trajpt_msg);

            

            
            trajset_msg.header.stamp = ros::Time::now();
            cmd_pub.publish(trajset_msg);
        
            

            
        }
        return;
    }

    void TopologyCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        message = msg->data;
        //cout<< message <<endl;
        //ROS_INFO("Received Bool message: %d", data);

    }

    
    void BoxCallback(const std_msgs::String msg)
    {
        
        

        vector<string> box_str;
        //spilited_str.resize(2);
        std::istringstream iss(msg.data);
        std::string substring;
        while (std::getline(iss, substring, ';'))
        {
            box_str.push_back(substring);
        }
        


        point_0.resize(5);
        point_0[0] = str2point(box_str[0]);
        point_0[1] = str2point(box_str[1]);
        point_0[2] = str2point(box_str[2]);
        point_0[3] = str2point(box_str[3]);
        point_0[4] = str2point(box_str[4]);
        //cout << "得到box5点"<<endl;

        get_5points = true;
    }

    void BoxindexCallback(const std_msgs::String msg)
    {
        if(!getstart&&get_obstacles == true&&get_now_position == true)
        {
            vector<string> box_index_str;
            //spilited_str.resize(2);
            std::istringstream iss(msg.data);
            std::string substring;
            while (std::getline(iss, substring, ';'))
            {
                box_index_str.push_back(substring);
            }
            start_point = str2point(box_index_str[0]);
            target_point_0 = start_point;
            /*start = now_position;
            goal  = target_point_0;
            path = AStar(start, goal, obstacles);*/
            path_point = start_point;

            getstart = true;
            //cout << "get start_point, path长度："<< path.size() <<endl;
        }
        
    }

    void Get_next_point_to_go()
    {
        
        if(getstart&&get_5points)
        {
           
            double height = point_0[4](2) - point_0[0](2);
            //if(  fabs(point_0[4].z()-now_position.z())<2 )//一个box扫描完毕
            /*if(  (Eigen::Vector3d(point_0[0](0),point_0[0](1),0.75*height+point_0[0](2))-now_position).norm()<6 )//一个box扫描完毕
            {
                height_index_0 = 0;
                point_index_0 = 0;
                cout<<"换盒-参数重置"<<endl;

                
            }*/
            
            if(point_index_0 == 10)
            {
                std_msgs::Int32 msg;
                msg.data = point_index_0;
                back_pub.publish(msg);
                cout<<"换盒-参数重置"<<endl;
                height_index_0 = 0;
                point_index_0 = 0;


            }
            
            
            if( (now_position - target_point_0).norm()<6 && inout==false )//飞机到达上一目标后
            {
                
                cout << "换点" << target_point_0 <<endl;
                path_index = 1;
                cout << "point_index_0："<< point_index_0 <<endl;
                /*target_point_0.x() = point_0[point_index_0-(point_index_0/4)*4](0);
                target_point_0.y() = point_0[point_index_0-(point_index_0/4)*4](1);
                //target_point_0.z() = point_0[point_index_0-(point_index_0/4)*4](2)+height/2*(height_index_0/4)+height/2;
                target_point_0.z() = point_0[point_index_0-(point_index_0/4)*4](2)+height/2;*/
                point_index_0 = point_index_0-(point_index_0/10)*10;
                if(point_index_0 == 0)
                {
                    target_point_0.x() = point_0[0](0);
                    target_point_0.y() = point_0[0](1);
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 1)
                {
                    target_point_0.x() = (point_0[0](0)+point_0[1](0))/2;
                    target_point_0.y() = (point_0[0](1)+point_0[1](1))/2;
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 2)
                {
                    target_point_0.x() = point_0[1](0);
                    target_point_0.y() = point_0[1](1);
                    target_point_0.z() = point_0[1](2)+height/2;
                }
                if(point_index_0 == 3)
                {
                    target_point_0.x() = (point_0[2](0)+point_0[1](0))/2;
                    target_point_0.y() = (point_0[2](1)+point_0[1](1))/2;
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 4)
                {
                    target_point_0.x() = point_0[2](0);
                    target_point_0.y() = point_0[2](1);
                    target_point_0.z() = point_0[2](2)+height/2;
                }
                if(point_index_0 == 5)
                {
                    target_point_0.x() = (point_0[2](0)+point_0[3](0))/2;
                    target_point_0.y() = (point_0[2](1)+point_0[3](1))/2;
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 6)
                {
                    target_point_0.x() = point_0[3](0);
                    target_point_0.y() = point_0[3](1);
                    target_point_0.z() = point_0[3](2)+height/2;
                }
                if(point_index_0 == 7)
                {
                    target_point_0.x() = (point_0[0](0)+point_0[3](0))/2;
                    target_point_0.y() = (point_0[0](1)+point_0[3](1))/2;
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 8)
                {
                    target_point_0.x() = point_0[0](0);
                    target_point_0.y() = point_0[0](1);
                    target_point_0.z() = point_0[0](2)+height/2;
                }
                if(point_index_0 == 9)
                {
                    target_point_0.x() = point_0[0](0);
                    target_point_0.y() = point_0[0](1);
                    target_point_0.z() = point_0[0](2)+height/2+height/4;
                }

                //---------------------------------------------------------------------
                if(point_index_0 == 0)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 0; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 1)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 0; // target pitch and yaw angle, respectively.

                }
                if(point_index_0 == 2)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 20; // target pitch and yaw angle, respectively                
                }
                if(point_index_0 == 3)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 20; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 4)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 40; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 5)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 40; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 6)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 60; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 7)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 60; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 8)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 60; // target pitch and yaw angle, respectively.
                }
                if(point_index_0 == 9)
                {
                    gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.
                    gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
                    gimbal_msg.linear.z = 60; // target pitch and yaw angle, respectively.
                }
                
                gimbal_msg.angular.x = 0.0;
                gimbal_msg.angular.y = 0; // in velocity control mode, this is the target pitch velocity
                gimbal_msg.angular.z = 0; 
                gimbal_pub_.publish(gimbal_msg);


                
                
                start = now_position;
                goal  = target_point_0;
                path = AStar(start, goal, obstacles);
                while(path.size()==0)
                {
                    cout <<"思考人生" << endl;
                    path = AStar(start, goal, obstacles);
                }


                if(obstacles.size()>3000)
                {
                    obstacles.clear();
                }

                
                cout << "path长度："<< path.size() <<endl;
                path_point = path[0];
                point_index_0++;
                height_index_0++;
                



                


                

                

                inout = true;
            }
            if( (now_position - target_point_0).norm()>1 && inout==true )
            {
                inout = false;
            }
            /*if(path.size()==0)
            {
                cout<< "不可到达，悬停并换点" <<endl;
                
                path_point = now_position;

                
            }*/

            
            if((now_position-path_point).norm()<0.5&&inout_flag == false)
            {
                cout <<"换下一个path点" <<endl;
                path_point = path[path_index];
                
                path_index++;
                inout_flag==true;

            }

            if( (now_position-path_point).norm()>0.3 && inout_flag==true )
            {
                inout_flag = false;
            }


            /*
            Eigen::Vector3d my_position = now_position;
            Eigen::Vector3d target_point = Eigen::Vector3d((point_0[0](0)+point_0[1](0)+point_0[2](0)+point_0[3](0))/4 ,(point_0[0](1)+point_0[1](1)+point_0[2](1)+point_0[3](1))/4 , (point_0[0](2)+point_0[1](2)+point_0[2](2)+point_0[3](2))/4+height/2 );
            //cout << "ddd"<<target_point <<endl;
            double distance = pow((pow((target_point[0]-my_position[0]),2) + pow((target_point[1]-my_position[1]),2)) , (1/2));
            double height_1 = target_point[2] - my_position[2] ;
            double pitch_angle = - atan(height_1 / distance);
            double yaw_angle = acos((target_point[0] - my_position[0]) / distance) - M_PI/2;
            gimbal_msg.linear.x = 1; // setting linear.x to -1.0 enables velocity control mode.

            gimbal_msg.linear.y = pitch_angle;
       
            gimbal_msg.linear.z = yaw_angle; 
            gimbal_msg.angular.x = 0.0;
            gimbal_msg.angular.y = 0.; // in velocity control mode, this is the target pitch velocity
            gimbal_msg.angular.z = 0.; 
            gimbal_pub_.publish(gimbal_msg);
            */
        

            


        }
        
        

    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        
        //obstacles.clear();

        
        /*vector<Eigen::Vector3d> point_vec;
        //initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        point_vec[0] = Eigen::Vector3d(initial_position.x()-2,initial_position.y()-2,initial_position.z()-2);
        point_vec[1] = Eigen::Vector3d(initial_position.x()+2,initial_position.y()-2,initial_position.z()-2);
        point_vec[2] = Eigen::Vector3d(initial_position.x()+2,initial_position.y()+2,initial_position.z()-2);
        point_vec[3] = Eigen::Vector3d(initial_position.x()-2,initial_position.y()+2,initial_position.z()-2);
        point_vec[4] = Eigen::Vector3d(initial_position.x()-2,initial_position.y()-2,initial_position.z()+2);
        point_vec[5] = Eigen::Vector3d(initial_position.x()+2,initial_position.y()-2,initial_position.z()+2);
        point_vec[6] = Eigen::Vector3d(initial_position.x()+2,initial_position.y()+2,initial_position.z()+2);
        point_vec[7] = Eigen::Vector3d(initial_position.x()-2,initial_position.y()+2,initial_position.z()+2);
        Box = Boundingbox(point_vec, 30);*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // 创建一个条件滤波器range_cond
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.5)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        condrem.filter (*cloud);

        // 使用removeNaNFromPointCloud函数来去除NaN点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        
        // 改稀疏一点,创建一个VoxelGrid滤波器
        // 执行滤波操作
        // 将新的点云数据添加到all_clouds中
        *all_clouds = *cloud;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (all_clouds);
        sor.setLeafSize (1.5f, 1.5f, 1.5f);

        // 执行滤波操作
        sor.filter (*all_clouds);

        for (size_t i = 0; i < all_clouds->points.size(); ++i)
            {
                // 获取点
                const auto& point = all_clouds->points[i];
                if(point.x >= now_position.x() - 10 && point.x <= now_position.x() + 10 &&
                   point.y >= now_position.y() - 10 && point.y <= now_position.y() + 10 &&
                   point.z >= now_position.z() - 10 && point.z <= now_position.z() + 10)
                {
                    obstacles.push_back(Obstacle(Eigen::Vector3d(point.x, point.y, point.z)));
                }
            }
        get_obstacles = true;
        //cout << "找到障碍物信息:" << obstacles.size() <<endl;
    }



    Eigen::Vector3d str2point(string input)
        {
            Eigen::Vector3d result;
            std::vector<string> value;
            boost::split(value, input, boost::is_any_of(","));//x,y,z
            // cout<<input<<endl;
            if (value.size() == 3)
            {
                result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
            }
            else
            {
                cout << input << endl;
                cout << "error use str2point 2" << endl;
            }
            return result;
        }


};