#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

struct position_info{
    Eigen::Vector3d position;
    bool update=false;
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


class gcs_task_assign{
    public:
        gcs_task_assign(){

        }
        gcs_task_assign(ros::NodeHandlePtr &nh_ptr_)//调用格式为“构造函数 : A(初始值),B(初始值),C(初始值)……”，如下，其中A、B、C分别是类的成员变量：

        : nh_ptr_(nh_ptr_)//相当于在构造函数中多一行nh_ptr = nh_ptr
        {
            namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
            position_pair["/jurong"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/raffles"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/changi"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/sentosa"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/nanyang"]={Eigen::Vector3d(0, 0, 1), false};
            //每个无人机位置


            xmax = -std::numeric_limits<double>::max();
            ymax = -std::numeric_limits<double>::max();
            zmax = -std::numeric_limits<double>::max();
            xmin = std::numeric_limits<double>::max();
            ymin = std::numeric_limits<double>::max();
            zmin = std::numeric_limits<double>::max();
            //------------------------------------------------------------------------

            client = nh_ptr_->serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
            cmd_pub_ = nh_ptr_->advertise<std_msgs::String>("/task_assign", 10); //gcs节点最后把任务分配信息发送到/task_assign/gcs
            srv.request.source = "gcs";
            srv.request.targets.push_back("all");
            srv.request.topic_name = "/task_assign";
            srv.request.package_name = "std_msgs";
            srv.request.message_type = "String";
            while (!serviceAvailable)
            {
                serviceAvailable = ros::service::waitForService("create_ppcom_topic", ros::Duration(10.0));
            }
            client.call(srv);
            //实现LOS，通过订阅/task_assign/gcs代替订阅/task_assign
            //-------------------------------------------------------------------------
            bbox_sub_ = nh_ptr_->subscribe<sensor_msgs::PointCloud>("/gcs/bounding_box_vertices", 10, &gcs_task_assign::bboxCallback, this);//调用subscribe是在类外面时，this换成&类名，类名::回调函数
            //边界框的顶点发布在此topic下
            agent_position_sub_=nh_ptr_->subscribe<std_msgs::String>("/broadcast/gcs", 10, &gcs_task_assign::positionCallback, this);
            //后续添加一个预测碰撞并改变路径的直接发送控制指令的函数
            //task_sub_ = nh_ptr->subscribe("/task_assign" + nh_ptr->getNamespace(), 10, &Agent::TaskCallback, this);
            //
            Agent_ensure_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerEnsureCB,     this);//0.1秒
            Massage_publish_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerMessageCB,     this);
        }
    private:
        ros::NodeHandlePtr nh_ptr_;
        
        //communication related param
        bool serviceAvailable = false;
        caric_mission::CreatePPComTopic srv;
        ros::ServiceClient client;

        ros::Publisher cmd_pub_;



        ros::Subscriber agent_position_sub_;
        ros::Subscriber bbox_sub_;
        
        ros::Timer Agent_ensure_Timer;
        ros::Timer Massage_publish_Timer;


        bool get_bbox=false;
        bool finish_box_order_generate=false;
        bool agent_info_get=false;

        list<string> namelist;
        map<string,position_info> position_pair;
        double update_time;

        double volumn_total=0;
        bool finish_bbox_record = false;
        vector<Boundingbox> box_set;
        vector<Boundingbox> box_set_0;
        vector<Boundingbox> box_set_1;
        
        vector<int> box_index_0;
        vector<int> box_index_1;
        vector<int> state_vec_0;
        vector<int> state_vec_1;

        Eigen::Vector3d start_point_0 ;  //jurong
        Eigen::Vector3d start_point_1;   //raffles

        double xmax;
        double ymax;
        double zmax;
        double xmin;
        double ymin;
        double zmin;
        double z_limit;

        int target_box_0=0;
        Eigen::Vector3d target_point_0;
        int point_index_0=0;
        vector<Eigen::Vector3d> point_0;
        int height_index_0=0;
        Eigen::Vector3d point_temp_0;


        int target_box_1=0;
        Eigen::Vector3d target_point_1;
        int point_index_1=0;
        vector<Eigen::Vector3d> point_1;
        int height_index_1=0;
        Eigen::Vector3d point_temp_1;


        vector<vector<string>> team_info;
        vector<vector<Boundingbox>> output_path;

        string result;

        void bboxCallback(const sensor_msgs::PointCloud::ConstPtr &msg) //一次性函数
        {
            if(!agent_info_get) //先完成TimerensureCB
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
            
            if(finish_box_order_generate)//记得后续自己写函数带上这个
            {
                return;
            }
            
            sensor_msgs::PointCloud cloud = *msg;  //sensor_msgs::PointCloud.points(类型：geometry_msgs/Point32[]数组)
            int num_points = cloud.points.size();  //vector.size()
            if (num_points % 8 == 0 && num_points > 8 * box_set.size())//box_set一开始为0，新来的点的数量可以构成box且后半条件不懂，难道不是一次全读进来？
            {
                volumn_total = 0;
                int num_box = num_points / 8;
                for (int i = 0; i < num_box; i++)
                {
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
                    double z_limit = (zmax+zmin)/2; //explorer上下分两半扫描
                    box_set.push_back(Boundingbox(point_vec, i));
                    volumn_total += box_set[i].getVolume();
                    point_vec.clear(); 
                }
                //-------------------------------------------------------------------添加部分-----------------------------
                int size = box_set.size();
                int plus_0 = 0;//下半区的box编号
                int plus_1 = 0;//上半区的box编号
                for(int i=0;i<size;i++)
                {
                    Eigen::Vector3d hign;
                    Eigen::Vector3d low;
                    Eigen::Vector3d mid;
                    vector<Eigen::Vector3d> vertice_temp;
                    vertice_temp = box_set[i].getVertices();
                    low = vertice_temp[0];                
                    hign = vertice_temp[4];
                    mid = Eigen::Vector3d(low(0), low(1), z_limit);

                    vector<Eigen::Vector3d> point_vec_0;
                    vector<Eigen::Vector3d> point_vec_1;
                    if(hign.z()>z_limit&&low.z()<z_limit)//判断该box是否需要中分
                    {            
                        double scale_cut = ((mid - vertice_temp[0]).norm())/((vertice_temp[4] - vertice_temp[0]).norm());
                        vector<Eigen::Vector3d> new_vertice_0(8);
                        vector<Eigen::Vector3d> new_vertice_1(8);
                        
                        new_vertice_0[0] = vertice_temp[0];
                        new_vertice_0[1] = vertice_temp[1];
                        new_vertice_0[2] = vertice_temp[2];
                        new_vertice_0[3] = vertice_temp[3];
                        new_vertice_0[4] = vertice_temp[0] + (vertice_temp[4] - vertice_temp[0]) * scale_cut;
                        new_vertice_0[5] = vertice_temp[1] + (vertice_temp[5] - vertice_temp[1]) * scale_cut;
                        new_vertice_0[6] = vertice_temp[2] + (vertice_temp[6] - vertice_temp[2]) * scale_cut;
                        new_vertice_0[7] = vertice_temp[3] + (vertice_temp[7] - vertice_temp[3]) * scale_cut;

                        new_vertice_1[0] = vertice_temp[0] + (vertice_temp[4] - vertice_temp[0]) * scale_cut;
                        new_vertice_1[1] = vertice_temp[1] + (vertice_temp[5] - vertice_temp[1]) * scale_cut;
                        new_vertice_1[2] = vertice_temp[2] + (vertice_temp[6] - vertice_temp[2]) * scale_cut;
                        new_vertice_1[3] = vertice_temp[3] + (vertice_temp[7] - vertice_temp[3]) * scale_cut;
                        new_vertice_1[4] = vertice_temp[4];
                        new_vertice_1[5] = vertice_temp[5];
                        new_vertice_1[6] = vertice_temp[6];
                        new_vertice_1[7] = vertice_temp[7];
                        for(int j=0;j<8;j++)
                        {
                            point_vec_0.push_back(new_vertice_0[j]);
                            point_vec_1.push_back(new_vertice_1[j]);
                        }

                        
                        box_set_0.push_back(Boundingbox(point_vec_0, plus_0));//可能存在隐患，如果后续需要用到编号
                        box_set_1.push_back(Boundingbox(point_vec_1, plus_1));
                        plus_0++;
                        plus_1++;
                        point_vec_0.clear(); 
                        point_vec_1.clear(); 
                    }
                    else if(hign.z()<=z_limit) //下半区
                    {
                        for(int j=0;j<8;j++)
                        {
                            point_vec_0.push_back(vertice_temp[j]);
                        }
                        box_set_0.push_back(Boundingbox(point_vec_0, plus_0));                       
                        plus_0++;
                        point_vec_0.clear(); 
                        point_vec_1.clear(); 
                    }
                    else if(low.z()>=z_limit)  //上半区
                    {
                        for(int j=0;j<8;j++)
                        {
                            point_vec_1.push_back(vertice_temp[j]);
                        }
                        box_set_1.push_back(Boundingbox(point_vec_1, plus_1));
                        plus_1++;
                        point_vec_0.clear(); 
                        point_vec_1.clear(); 
                    }
                }
                //分成了两个box_set:box_set_0、box_set_1（也是后续唯二有用的变量）
                finish_bbox_record = true;//第一次完成函数后bbox就标记完了，此函数不再运行
                cout<<"finish_bbox_record"<<endl;
                //-------------------------------------------------------------------添加部分-----------------------------
                //Team allocate
                //Team_allocate();
                Get_search_order();
                //Use best first to generate the global trajactory
                //Best_first_search();
                //Clip the best path
                //Clip_the_task();
                //Generate the massage
                //generate_massage();


            }
            else
            {
                return;
            }
            return;
        }

        //函数用来init_pos
        void positionCallback(const std_msgs::String msg) //msg的形式belike：init_pos;origin;position_str
        {
            istringstream str(msg.data);//把string里的内容传给type、origin、position_str
            string type;
            getline(str,type,';');
            if(type=="init_pos")
            {
                string origin;
                getline(str,origin,';');
                string position_str;
                getline(str,position_str,';');
                if(!position_pair[origin].update){//该无人机（origin）位置信息若未更新则更新，用完后应重新变为false
                    update_time=ros::Time::now().toSec(); //最近的位置信息更新时间
                    position_pair[origin].position=str2point(position_str);
                    position_pair[origin].update=true;
                }
            }
            //cout<<"prepare_for_find_next"<<endl;
            Get_next_point_to_go_0();
            Get_next_point_to_go_1();
        }

        void TimerEnsureCB(const ros::TimerEvent &)//
        {   
            if(agent_info_get)
            {
                return;
            }
            bool finish_agent_record=false; 
            for(auto& name:namelist)
            {
                if(position_pair[name].update){
                    finish_agent_record=true;   //至少有一个位置信息更新了就把此标记改为ture
                    break;
                }
            }
            if(!finish_agent_record)
            {
                return;
            }
            double time_now=ros::Time::now().toSec();

            if(fabs(time_now-update_time)>10) //等十秒，不知为何？
            {
                agent_info_get=true;
            }
        }

        


        Eigen::Vector3d str2point(string input)
        {
            Eigen::Vector3d result;
            std::vector<string> value;
            boost::split(value, input, boost::is_any_of(","));
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

    //如果转圈过程中检测到有点云离自己很近（设置一个FOV）就抬高后再旋转    
    //找出发点 我们要先让一个升空后再用两次这个函数的变形
    void Get_search_order(){//找最近的box顶点作为起点

        start_point_0 = position_pair["/jurong"].position;
        start_point_1 = position_pair["/raffles"].position;
        while(box_index_0.size()<box_set_0.size()){
            int index;//
            int state;//第几个点
            double mindis=std::numeric_limits<double>::max();
            for(int i=0;i<box_set_0.size();i++){
                if(find(box_index_0.begin(),box_index_0.end(),i)!=box_index_0.end()&&box_index_0.size()>0){
                    continue;
                }
                for(int j=0;j<8;j++){  
                    double dis=(box_set_0[i].getVertices()[j]-start_point_0).norm();
                    if(dis<mindis){
                        mindis=dis;
                        state=j;
                        index=i;
                    }
                }
            }
            box_index_0.push_back(index);
            state_vec_0.push_back(state);
            start_point_0=box_set_0[box_index_0[0]].getVertices()[0];
            target_point_0 = start_point_0;
        }
        while(box_index_1.size()<box_set_1.size()){
            int index;
            int state;
            double mindis=std::numeric_limits<double>::max();
            for(int i=0;i<box_set_1.size();i++){
                if(find(box_index_1.begin(),box_index_1.end(),i)!=box_index_1.end()&&box_index_1.size()>0){//find作用于容器就是这个用法，!=意思是找到了，第一次之后第二个条件就满足了
                    continue;//进入下一次
                }
                for(int j=0;j<8;j++){  
                    double dis=(box_set_1[i].getVertices()[j]-start_point_1).norm();
                    if(dis<mindis){
                        mindis=dis;
                        state=j;
                        index=i;
                    }
                }
            }
            box_index_1.push_back(index);//
            state_vec_1.push_back(state);
            start_point_1=box_set_1[box_index_1[0]].getVertices()[0];
            target_point_1 = start_point_1;
        }
        //按离起始点的距离把盒子们排好序（不需要raffles原地起飞）
        //后续有用的是box_index_0和box_index_1，用法如下box_set_0[box_index_0[0]]直到box_set_0[box_index_0[box_index_0.size()]]
        //是0（jurong）、1（raffles）分别要走的顺序
        finish_box_order_generate=true;
        cout<<"finish_box_order_generate"<<endl;

    }




    void TimerMessageCB(const ros::TimerEvent &)
    {   
        if(finish_box_order_generate)
        {
            std_msgs::String task;
            task.data=to_string(target_point_0.x())+","+to_string(target_point_0.y())+","+to_string(target_point_0.z())+";"+to_string(target_point_1.x())+","+to_string(target_point_1.y())+","+to_string(target_point_1.z());
            cmd_pub_.publish(task);
            cout<<"finish_pub"<<endl;
        }
    }


    void Get_next_point_to_go_0()
    {
        if(!finish_box_order_generate)//初始化之后再考虑这些
        {
            return;
        }
        cout<<"axiba1"<<endl;
        point_temp_0 = box_set_0[box_index_0[target_box_0]].getVertices()[7];
        cout<<"axiba"<<endl;
        if(position_pair["/jurong"].position.z() == point_temp_0.z())//一个box扫描完毕
        {
            if(target_box_0+1<box_index_0.size())//~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            {
                target_box_0++;//不超过最多盒数
                height_index_0 = 0;

            }
            else
            {
                return;
            }
        }
        
        
        point_0[0] = box_set_0[box_index_0[target_box_0]].getVertices()[0];
        point_0[1] = box_set_0[box_index_0[target_box_0]].getVertices()[1];
        point_0[2] = box_set_0[box_index_0[target_box_0]].getVertices()[2];
        point_0[3] = box_set_0[box_index_0[target_box_0]].getVertices()[3];
        point_0[4] = box_set_0[box_index_0[target_box_0]].getVertices()[4];

        double height = point_0[4](2) - point_0[0](2);
        
        if(position_pair["/jurong"].position == target_point_0)//飞机到达上一目标后
        {
            if(point_index_0>=4)
            {
                height_index_0++;
                point_index_0 = 0;
                target_point_0.x() = point_0[point_index_0](0);
                target_point_0.y() = point_0[point_index_0](1);
                target_point_0.z() = point_0[point_index_0](2)+2*height_index_0;  
                point_index_0++;
            }
            if(point_index_0<=3)
            {
                target_point_0.x() = point_0[point_index_0](0);
                target_point_0.y() = point_0[point_index_0](1);
                target_point_0.z() = point_0[point_index_0](2);
                point_index_0++;
            }

        }
    }

    void Get_next_point_to_go_1()
    {
        if(!finish_box_order_generate)//初始化之后再考虑这些
        {
            return;
        }
        cout<<"axiba1"<<endl;
        point_temp_1 = box_set_1[box_index_1[target_box_1]].getVertices()[7];
        if(position_pair["/raffles"].position.z() == point_temp_1.z())//一个box扫描完毕
        {
            if(target_box_1+1<box_index_1.size())
            {
                target_box_1++;//不超过最多盒数
                height_index_1 = 0;

            }
            else
            {
                return;
            }
        }
        
        point_1[0] = box_set_1[box_index_1[target_box_1]].getVertices()[0];
        point_1[1] = box_set_1[box_index_1[target_box_1]].getVertices()[1];
        point_1[2] = box_set_1[box_index_1[target_box_1]].getVertices()[2];
        point_1[3] = box_set_1[box_index_1[target_box_1]].getVertices()[3];
        point_1[4] = box_set_1[box_index_1[target_box_1]].getVertices()[4];

        double height = point_1[4](2) - point_1[0](2);
        
        if(position_pair["/raffles"].position == target_point_1)//飞机到达上一目标后
        {
            if(point_index_1>=4)
            {
                height_index_1++;
                point_index_1 = 0;
                target_point_1.x() = point_1[point_index_1](0);
                target_point_1.y() = point_1[point_index_1](1);
                target_point_1.z() = point_1[point_index_1](2)+2*height_index_1;  
                point_index_1++;
            }
            if(point_index_1<=3)
            {
                target_point_1.x() = point_1[point_index_1](0);
                target_point_1.y() = point_1[point_index_1](1);
                target_point_1.z() = point_1[point_index_1](2);
                point_index_1++;
            }

        }
    }

};