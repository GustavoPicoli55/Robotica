#include "Perception.h"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cstring>

Perception::Perception(float mapWidth, float mapHeight, float cellSize):
    mapWidth_(mapWidth), mapHeight_(mapHeight), scale_(1.0/cellSize)
{
    // Initialize grids
    numCellsX_ = mapWidth_*scale_;
    numCellsY_ = mapHeight_*scale_;

    gridLaserLogOdds_.resize(numCellsX_*numCellsY_, 0); // default value = log  0.5/(1-0.5) = 0
    gridLaserHIMM_.resize(numCellsX_*numCellsY_, 7);    // default value = 7 (half between 0 and 15)
    gridSonar_.resize(numCellsX_*numCellsY_, 7);    // default value = 7

    // Initialize OccupancyGrid messages

    // STRUCTURE OF nav_msgs::msg::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    msg_mapLaserHIMM_.header.frame_id="odom";
    msg_mapLaserHIMM_.info.height = numCellsY_;
    msg_mapLaserHIMM_.info.width = numCellsX_;
    msg_mapLaserHIMM_.info.resolution = cellSize;
    msg_mapLaserHIMM_.info.origin.position.x = -mapWidth/2.0;
    msg_mapLaserHIMM_.info.origin.position.y = -mapHeight/2.0;
    msg_mapLaserHIMM_.data.resize(numCellsX_*numCellsY_, -1);

    msg_mapLaserLogOdds_.header.frame_id="odom";
    msg_mapLaserLogOdds_.info = msg_mapLaserHIMM_.info;
    msg_mapLaserLogOdds_.data.resize(numCellsX_*numCellsY_, -1);

    msg_mapSonar_.header.frame_id="odom";
    msg_mapSonar_.info = msg_mapLaserHIMM_.info;
    msg_mapSonar_.data.resize(numCellsX_*numCellsY_, -1);

}

/////////////////////////////
/// Funcoes de mapeamento ///
/////////////////////////////
nav_msgs::msg::OccupancyGrid& Perception::updateMapLaserWithLogOdds(const std::vector<float>& z, const Pose2D& robot)
{
    int rx = robot.x * scale_;
    int ry = robot.y * scale_;

    float maxRange = 10.0;  // 10 m
    float lambda_r = 0.2;   //  20 cm
    float lambda_phi = 1.0; // 1 degree
    int maxRangeInt = maxRange * scale_;
    const int HIMM_MAX = 15;
    const int HIMM_MIN = 0;

    const float P_OCC = 0.56; // probabilidade de ocupação
    const float P_FREE = 0.45; // probabilidade de livre

    // varrer celulas ao redor de (rx,ry) em um range de -maxRangeInt ate +maxRangeInt nas duas direcoes
    // e entao atualizar celulas dentro do campo de visao dos sensores 
    for (int x = rx - maxRangeInt; x <= rx + maxRangeInt; ++x)
    {
        for (int y = ry - maxRangeInt; y <= ry + maxRangeInt; ++y)
        {
            // 1. Computar a orientação da célula em relação ao robô
            float phi = normalizeAngleDEG(RAD2DEG(atan2(y - ry, x - rx)) - robot.theta);
            int k = getNearestLaserBeam(phi); // 3. Encontrar o feixe mais próximo
            float phi_k = getAngleOfLaserBeam(k);

            // 1. Computar distância r até a célula (convertida para metros)
            float r = hypot(x - rx, y - ry) / scale_;
            float z_k = z[k]; // leitura do sensor para o feixe k

            // 4. Verifica se está dentro do campo de visão e do alcance
            if (fabs(phi_k - phi) > lambda_phi || r > std::min(maxRange, static_cast<float>(z_k + lambda_r / 2.0)))
                continue;

            float prob = 0.5;
            // Define se a célula é ocupada ou livre
            if (z_k < maxRange && fabs(r - z_k) < lambda_r / 2.0)
                prob = P_OCC;
            else if (r < z_k)
                prob = P_FREE;

            int idx = getCellIndexFromXY(x, y); // índice da célula no vetor 1D
            gridLaserLogOdds_[idx] += getLogOddsFromLikelihood(prob); // atualiza o valor de log-odds
            msg_mapLaserLogOdds_.data[idx] = static_cast<int8_t>(getLikelihoodFromLogOdds(gridLaserLogOdds_[idx]) * 100); // conversão para visualização no RViz
        }
    }

    return msg_mapLaserLogOdds_;
}

nav_msgs::msg::OccupancyGrid& Perception::updateMapLaserWithHIMM(const std::vector<float>& z, const Pose2D& robot)
{
    int rx = robot.x * scale_;
    int ry = robot.y * scale_;

    float maxRange = 10.0;  // 10 m
    float lambda_r = 0.2;   //  20 cm
    float lambda_phi = 1.0; // 1 degree
    int maxRangeInt = maxRange * scale_;
    const int HIMM_MAX = 15;
    const int HIMM_MIN = 0;

    // varrer celulas ao redor de (rx,ry) em um range de -maxRangeInt ate +maxRangeInt nas duas direcoes
    for (int x = rx - maxRangeInt; x <= rx + maxRangeInt; ++x)
    {
        for (int y = ry - maxRangeInt; y <= ry + maxRangeInt; ++y)
        {
            // 2. Computar a orientação da célula em relação ao robô
            float phi = normalizeAngleDEG(RAD2DEG(atan2(y - ry, x - rx)) - robot.theta);
            int k = getNearestLaserBeam(phi); // 3. Obter feixe mais próximo
            float phi_k = getAngleOfLaserBeam(k);

            // 1. Computar a distância r até a célula
            float r = hypot(x - rx, y - ry) / scale_;
            float z_k = z[k];

            // 4. Verificar se está dentro da abertura do feixe e alcance do sensor
            if (fabs(phi_k - phi) > lambda_phi || r > std::min(maxRange, static_cast<float>(z_k + lambda_r / 2.0)))
                continue;

            int idx = getCellIndexFromXY(x, y);
            // Atualiza valores HIMM
            if (r < z_k - lambda_r)
                gridLaserHIMM_[idx] = std::max(gridLaserHIMM_[idx] - 1, HIMM_MIN);
            else if (fabs(r - z_k) <= lambda_r)
                gridLaserHIMM_[idx] = std::min(gridLaserHIMM_[idx] + 3, HIMM_MAX);

            msg_mapLaserHIMM_.data[idx] = static_cast<int8_t>((gridLaserHIMM_[idx] * 100) / HIMM_MAX);
        }
    }

    return msg_mapLaserHIMM_;
}

nav_msgs::msg::OccupancyGrid& Perception::updateMapSonar(const std::vector<float>& z, const Pose2D& robot)
{
    int rx = robot.x * scale_;
    int ry = robot.y * scale_;

    float maxRange = 5.0; // 5 m
    float lambda_r = 0.5; //  50 cm
    const int HIMM_MAX = 15;
    const int HIMM_MIN = 0;

    // varrer as celulas no eixo acustico de cada leitura do sonar usando um algoritmo de rasterização de linhas (ex: DDA)
    for (int k = 0; k < 8; ++k)
    {
        float phi = normalizeAngleDEG(getAngleOfSonarBeam(k) + robot.theta);
        float z_k = z[k] * scale_;

        // Calcula o ponto final da linha do feixe do sonar
        int endX = rx + maxRange * scale_ * cos(DEG2RAD(phi));
        int endY = ry + maxRange * scale_ * sin(DEG2RAD(phi));

        // Rasterização da linha usando DDA
        int dx = endX - rx;
        int dy = endY - ry;
        int steps = std::max(abs(dx), abs(dy));

        float x_inc = dx / static_cast<float>(steps);
        float y_inc = dy / static_cast<float>(steps);

        float x = rx;
        float y = ry;

        for (int i = 0; i <= steps; ++i)
        {
            int xi = round(x);
            int yi = round(y);
            int idx = getCellIndexFromXY(xi, yi);
            float dist = hypot(xi - rx, yi - ry);

            // Atualiza as células como livres ou ocupadas de acordo com a leitura do sonar
            if (dist < z_k - lambda_r / 2.0f)
                gridSonar_[idx] = std::max(gridSonar_[idx] - 1.0f, static_cast<float>(HIMM_MIN));
            else if (dist <= z_k + lambda_r / 2.0f)
                gridSonar_[idx] = std::min(gridSonar_[idx] + 3.0f, static_cast<float>(HIMM_MAX));
            else
                break;

            // Conversão para formato de visualização no RViz
            msg_mapSonar_.data[idx] = static_cast<int8_t>((gridSonar_[idx] * 100) / HIMM_MAX);

            x += x_inc;
            y += y_inc;
        }
    }

    return msg_mapSonar_;
}

//////////////////////////
/// Funcoes Auxiliares ///
//////////////////////////

int Perception::getCellIndexFromXY(int x, int y)
{
    int cellX = numCellsX_/2 + x;
    int cellY = numCellsY_/2 + y;

    return cellX + numCellsX_*cellY;
}

float sonarAngles_[8] = {90, 50, 30, 10, -10, -30, -50, -90};

float Perception::getAngleOfSonarBeam(int k)
{
    return sonarAngles_[k];
}

int Perception::getNearestSonarBeam(float angle)
{
    if(angle>70.0)
        return 0;
    else if(angle>40 && angle<=70)
        return 1;
    else if(angle>20 && angle<=40)
        return 2;
    else if(angle>0 && angle<=20)
        return 3;
    else if(angle>-20 && angle<=0)
        return 4;
    else if(angle>-40 && angle<=-20)
        return 5;
    else if(angle>-70 && angle<=-40)
        return 6;
    else //if(angle<=-70.0)
        return 7;

}

float Perception::getAngleOfLaserBeam(int k)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    return 90.0-(float)k;
}

int Perception::getNearestLaserBeam(float angle)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    if(angle>90.0)
        return 0;
    else if(angle<-90.0)
        return 180;
    else{
        return 90-(int)((angle > 0.0)?(angle + 0.5):(angle - 0.5));
    }
}

/////////////////////////////////////////////////
/// Callbacks dos topicos do LASER e do SONAR ///
/////////////////////////////////////////////////

void Perception::receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &value)
{
    //  STRUCTURE OF nav_msgs::msg::Odometry

    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    
    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    odomROS.header = value->header;
    
    odomROS.child_frame_id = value->child_frame_id;
    
    odomROS.pose = value->pose;

    odomROS.twist = value->twist;
}


void Perception::receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value)
{
    //  STRUCTURE OF sensor_msgs::msg::LaserScan

    // Single scan from a planar laser range-finder
    // 
    // If you have another ranging device with different behavior (e.g. a sonar
    // array), please find or create a different message, since applications
    // will make fairly laser-specific assumptions about this data

    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    laserROS.header = value->header;

    // float32 angle_min        # start angle of the scan [rad]
    // float32 angle_max        # end angle of the scan [rad]
    // float32 angle_increment  # angular distance between measurements [rad]
    laserROS.angle_min = value->angle_min;
    laserROS.angle_max = value->angle_max;
    laserROS.angle_increment = value->angle_increment;

    // float32 time_increment   # time between measurements [seconds] - if your scanner
    //                          # is moving, this will be used in interpolating position
    //                          # of 3d points
    // float32 scan_time        # time between scans [seconds]
    laserROS.time_increment = value->time_increment;
    laserROS.scan_time = value->scan_time;

    // float32 range_min        # minimum range value [m]
    // float32 range_max        # maximum range value [m]
    laserROS.range_min = value->range_min;
    laserROS.range_max = value->range_max;

    // float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    // float32[] intensities    # intensity data [device-specific units].  If your
    //                          # device does not provide intensities, please leave
    //                          # the array empty.
    laserROS.ranges = value->ranges;
    laserROS.intensities = value->intensities;
}

void Perception::receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value)
{
    //  STRUCTURE OF sensor_msgs::msg::PointCloud2

    // This message holds a collection of N-dimensional points, which may
    // contain additional information such as normals, intensity, etc. The
    // point data is stored as a binary blob, its layout described by the
    // contents of the "fields" array.

    // The point cloud data may be organized 2d (image-like) or 1d
    // (unordered). Point clouds organized as 2d images may be produced by
    // camera depth sensors such as stereo or time-of-flight.

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    sonarROS.header = value->header;

    // # 2D structure of the point cloud. If the cloud is unordered, height is
    // # 1 and width is the length of the point cloud.
    // uint32 height
    // uint32 width
    sonarROS.height = value->height;
    sonarROS.width = value->width;

    // # Describes the channels and their layout in the binary data blob.
    // PointField[] fields
    sonarROS.fields = value->fields;

    // bool    is_bigendian # Is this data bigendian?
    // uint32  point_step   # Length of a point in bytes
    // uint32  row_step     # Length of a row in bytes
    sonarROS.is_bigendian = value->is_bigendian;
    sonarROS.point_step = value->point_step;
    sonarROS.row_step = value->row_step;

    // uint8[] data         # Actual point data, size is (row_step*height)
    sonarROS.data = value->data;

    // bool is_dense        # True if there are no invalid points
    sonarROS.is_dense = value->is_dense;
}

Pose2D Perception::getLatestPoseFromOdometry()
{
    Pose2D robotPose;

    // Update robotPose from robot transformation
    robotPose.x = odomROS.pose.pose.position.x;
    robotPose.y = odomROS.pose.pose.position.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(odomROS.pose.pose.orientation.x,
                       odomROS.pose.pose.orientation.y, 
                       odomROS.pose.pose.orientation.z, 
                       odomROS.pose.pose.orientation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    // Update orientation with yaw
    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}

std::vector<float> Perception::getLatestLaserRanges()
{
    int numLasers = laserROS.ranges.size();

    std::vector<float> lasers(numLasers);

    //    std::cout << "LASER: " << numLasers << std::endl;
    for (int i = 0; i < numLasers; i++)
    {
        lasers[i] = laserROS.ranges[numLasers - i - 1];
        if (lasers[i] < 0)
            lasers[i] = 32.0; // max range from rosaria
    }

    return lasers;
}

std::vector<float> Perception::getLatestSonarRanges()
{
    //int numbytes = sonarROS.data.size();
    //int numfields = sonarROS.fields.size();
    int numSonars = sonarROS.width;

    float x,y,z;
    
    std::vector<float> sonars(numSonars);

    // std::cout << "SONAR: " << numSonars << std::endl;
    for(int n=0; n<numSonars; n++){
        memcpy (&x, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[0].offset], sizeof (float));
        memcpy (&y, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[1].offset], sizeof (float));
        memcpy (&z, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[2].offset], sizeof (float));        
        sonars[n] = sqrt(pow(x,2.0)+pow(y,2.0));
        // std::cout << "ang: " << RAD2DEG(atan2(y,x)) << " range: " << sonars[n] << ' ';
    }
    // std::cout << std::endl;

    return sonars;
}
