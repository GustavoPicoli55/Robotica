#include "Perception.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>

Perception::Perception()
{
    receivedMap_ = false;
    startedMCL_ = false;

    numParticles_=10000;
    maxRange_ = 10.0;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_ = new std::default_random_engine(seed); 
}

///////////////////////////////////////
/// Funcoes do filtro de particulas ///
///////////////////////////////////////

void Perception::MCL_sampling(const Action &u){
    /// TODO: propagar todas as particulas de acordo com o modelo de movimento baseado em odometria

    /// Odometria definida pela estrutura Action, composta por 3 variaveis double:
    /// rot1, trans e rot2
    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    /// Seguindo o modelo de Thrun, devemos gerar 3 distribuicoes normais, uma para cada componente da odometria

    /// Para definir uma distribuição normal X de media M e variancia V, pode-se usar:
    // std::normal_distribution<double> samplerX(M,V);
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerX(*generator_)
    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)

    /// Para acessar a i-ésima particula, usar:
    // particles_[i].p.x
    // particles_[i].p.y
    // particles_[i].p.theta

    // Estes coeficientes controlam o ruído aplicado às ações do robô
    const double ROT_NOISE_GAIN = 0.7;   // ruído proporcional à rotação
    const double TRANS_NOISE_GAIN = 0.5; // ruído proporcional à translação

    
    // Gera ruído normal com média zero e variância proporcional às ações
    std::normal_distribution<double> dist_rot1(0.0, ROT_NOISE_GAIN * std::abs(u.rot1) + TRANS_NOISE_GAIN * u.trans);
    std::normal_distribution<double> dist_trans(0.0, TRANS_NOISE_GAIN * u.trans + ROT_NOISE_GAIN * (std::abs(u.rot1) + std::abs(u.rot2)));
    std::normal_distribution<double> dist_rot2(0.0, ROT_NOISE_GAIN * std::abs(u.rot2) + TRANS_NOISE_GAIN * u.trans);
    
    // Atualização de todas as partículas
    for (Particle &particle : particles_){
        Pose2D &pose = particle.p;

        // Aplica ruído nas ações do robô
        double noisy_rot1 = u.rot1 + dist_rot1(*generator_);
        double noisy_trans = u.trans + dist_trans(*generator_);
        double noisy_rot2 = u.rot2 + dist_rot2(*generator_);

        // Cálculo da nova pose com base no movimento amostrado
        double heading = pose.theta + noisy_rot1;
        pose.x += noisy_trans * std::cos(heading);
        pose.y += noisy_trans * std::sin(heading);
        pose.theta += noisy_rot1 + noisy_rot2;
    }
}

void Perception::MCL_weighting(const std::vector<float> &z){
   /// TODO: faça a pesagem de todas as particulas

    /// 1) elimine particulas fora do espaco livre, dando peso 0
    //       para achar a celula correspondente no grid, compute o indice dela
    //          unsigned int ix = particles_[i].p.x*scale_;
    //          unsigned int iy = particles_[i].p.y*scale_;
    //          unsigned int indice = ix + iy*numCellsX_;
    //       entao teste gridMap_.data[indice] <-- espaco livre tem valor 0

    /// 2) compare as observacoes da particula com as observacoes z do robo e pese-as
    //       Use a funcao computeExpectedMeasurement(k, particles[i].p)
    //       para achar a k-th observacao esperada da particula i
    ///    ao fim, normalize os pesos
    // Número total de feixes do laser (o laser possui 181 leituras de 1 grau cada)
    const int TOTAL_BEAMS = 181;

    // Parâmetros do modelo de sensor Gaussiano
    const double SENSOR_VARIANCE = 7.5;  // variância aumentada para permitir mais tolerância
    const double GAUSS_COEFF = 1.0 / std::sqrt(2.0 * M_PI * SENSOR_VARIANCE);

    double total_weight = 0.0;

    // Avalia cada partícula 
    for (Particle &particle : particles_){
        Pose2D &pose = particle.p;

        // Conversão da posição para índice na grade de ocupação
        unsigned int x_idx = static_cast<unsigned int>(pose.x * scale_);
        unsigned int y_idx = static_cast<unsigned int>(pose.y * scale_);
        unsigned int cell_index = x_idx + y_idx * numCellsX_;

        // Verifica se a partícula está em uma célula livre
        if (gridMap_.data[cell_index] != 0){
            // Fora do espaço livre → peso nulo
            particle.w = 0.0;
            continue;
        }

        // Cálcula da probabilidade com base na leitura
        double likelihood = 1.0;

        // Para acelerar: usa uma amostragem de feixes a cada 10
        for (int beam = 0; beam < TOTAL_BEAMS; beam += 10){
            // Leitura esperada da partícula (ray casting)
            double expected = computeExpectedMeasurement(beam, pose);
            double observed = z[beam];

            // Modelo Gaussiano simplificado (probabilidade da leitura)
            double diff = observed - expected;
            double exponent = -(diff * diff) / (2.0 * SENSOR_VARIANCE);
            double prob = GAUSS_COEFF * std::exp(exponent);

            likelihood *= prob;
        }

        particle.w = likelihood;
        total_weight += likelihood;
    }

    // Normalização dos pesos
    if (total_weight > 0.0){
        for (Particle &particle : particles_)
            particle.w /= total_weight;
    }
    else{
        // Se todas as partículas tiveram peso zero, redistribui pesos uniformemente
        double uniform_weight = 1.0 / static_cast<double>(particles_.size());
        for (Particle &particle : particles_)
            particle.w = uniform_weight;
    }

}

void Perception::MCL_resampling(){
    // gere uma nova geração de particulas com o mesmo tamanho do conjunto atual
    std::vector<Particle> nextGeneration;

    /// TODO: Implemente o Low Variance Resampling

    /// Para gerar amostras de uma distribuição uniforme entre valores MIN e MAX, pode-se usar:
    // std::uniform_real_distribution<double> samplerU(MIN,MAX));
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerU(*generator_)
    /// onde *generator_ é um gerador de numeros aleatorios (definido no construtor da classe)
    int num_particles = numParticles_;
    nextGeneration.reserve(num_particles);

    // Amostrador de baixa variância (Low-Variance Resampler)

    std::uniform_real_distribution<double> uni_dist(0.0, 1.0 / num_particles);
    double r = uni_dist(*generator_); // ponto inicial aleatório
    double c = particles_[0].w;       // acumulador de pesos
    int i = 0;

    // Seleção de partículas com sobreposição
    for (int m = 0; m < num_particles; ++m){
        double U = r + m * (1.0 / num_particles); // alvo atual no intervalo [0,1]

        // Encontra a partícula correspondente acumulando pesos
        while (U > c){
            i++;
            if (i >= num_particles) {
                i = num_particles - 1; // previne index out of bounds
                break;
            }
            c += particles_[i].w;
        }

        nextGeneration.push_back(particles_[i]); // seleciona partícula atual
    }

    // ===============================
    // Substitui o vetor de partículas
    particles_ = nextGeneration;

    // Verificação de integridade
    std::cout << "[Resampling] Nova geração de partículas: " << particles_.size() << std::endl;
    assert(particles_.size() == static_cast<size_t>(numParticles_));
}

/////////////////////////////////////////////////////////////////////////////
/// Funcoes de inicializacao e funcoes auxiliares do filtro de particulas ///
/////////////////////////////////////////////////////////////////////////////

void Perception::MCL_initialize()
{

    int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;
    minKnownX_ = numCellsX_-1;
    minKnownY_ = numCellsY_-1;
    maxKnownX_ = maxKnownY_ = 0;

    // Update known limits
    for(int x=0; x<numCellsX_; x++){
        for(int y=0; y<numCellsY_; y++){
            unsigned int i = x + y*numCellsX_;
            if(gridMap_.data[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;

                if(gridMap_.data[i]>-1 && gridMap_.data[i]<90)
                    gridMap_.data[i]=0;
                else
                    gridMap_.data[i]=100;
            }
        }
    }

    particles_.resize(numParticles_);

    std::uniform_real_distribution<double> randomX(minKnownX_/scale_,maxKnownX_/scale_);
    std::uniform_real_distribution<double> randomY(minKnownY_/scale_,maxKnownY_/scale_);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial particles set
    for(int i=0; i<numParticles_; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles_[i].p.x = randomX(*generator_);
            particles_[i].p.y = randomY(*generator_);
            particles_[i].p.theta = randomTh(*generator_);

            // check if particle is valid (known and not obstacle)
            unsigned int ix = particles_[i].p.x*scale_;
            unsigned int iy = particles_[i].p.y*scale_;
            if(gridMap_.data[ix + iy*numCellsX_] == 0)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles_[i].p.x << ' '
                  << particles_[i].p.y << ' '
                  << RAD2DEG(particles_[i].p.theta) << std::endl;
    }


    startedMCL_=true;
}

float Perception::computeExpectedMeasurement(int rangeIndex, Pose2D &pose)
{
    double angle = pose.theta + double(90-rangeIndex)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange_;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange_;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange_;
    }

    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale_;

    double i=pose.x*scale_;
    double j=pose.y*scale_;
    for(int k=0;k<(int)(dist);k++){

        if(gridMap_.data[(int)i + (int)j*numCellsX_] == 100){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale_-(i+deltaX),2)+pow(pose.y*scale_-(j+deltaY),2))/scale_;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange_;
}

void Perception::MCL_updateParticles(geometry_msgs::msg::PoseArray& msg_particles, rclcpp::Time now)
{
    msg_particles.header.frame_id="map";
    msg_particles.header.stamp = now;

    msg_particles.poses.resize(numParticles_);
    for(int i=0; i<numParticles_; i++){
        msg_particles.poses[i].position.x = particles_[i].p.x + mapOrigin_.x;
        msg_particles.poses[i].position.y = particles_[i].p.y + mapOrigin_.y;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, particles_[i].p.theta );
        msg_particles.poses[i].orientation = tf2::toMsg(quat_tf);
    }
}

/////////////////////////////////////////////////
/// Callbacks dos topicos do LASER e do SONAR ///
/////////////////////////////////////////////////

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
    laserROS_.header = value->header;

    // float32 angle_min        # start angle of the scan [rad]
    // float32 angle_max        # end angle of the scan [rad]
    // float32 angle_increment  # angular distance between measurements [rad]
    laserROS_.angle_min = value->angle_min;
    laserROS_.angle_max = value->angle_max;
    laserROS_.angle_increment = value->angle_increment;

    // float32 time_increment   # time between measurements [seconds] - if your scanner
    //                          # is moving, this will be used in interpolating position
    //                          # of 3d points
    // float32 scan_time        # time between scans [seconds]
    laserROS_.time_increment = value->time_increment;
    laserROS_.scan_time = value->scan_time;

    // float32 range_min        # minimum range value [m]
    // float32 range_max        # maximum range value [m]
    laserROS_.range_min = value->range_min;
    laserROS_.range_max = value->range_max;

    // float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    // float32[] intensities    # intensity data [device-specific units].  If your
    //                          # device does not provide intensities, please leave
    //                          # the array empty.
    laserROS_.ranges = value->ranges;
    laserROS_.intensities = value->intensities;
}

std::vector<float> Perception::getLatestLaserRanges()
{
    int numLasers = laserROS_.ranges.size();

    std::vector<float> lasers(numLasers);

    //    std::cout << "LASER: " << numLasers << std::endl;
    for (int i = 0; i < numLasers; i++)
    {
        lasers[i] = laserROS_.ranges[numLasers - i - 1];
        if (lasers[i] < 0)
            lasers[i] = 32.0; // max range from rosaria
    }

    return lasers;
}

void Perception::receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value)
{
    // STRUCTURE OF nav_msgs::OccupancyGrid
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

    if(receivedMap_==false){
        gridMap_.header = value->header;
        gridMap_.info = value->info;
        gridMap_.data = value->data;

        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;
        scale_ = 1.0/cellSize;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;

        mapOrigin_.x = value->info.origin.position.x;
        mapOrigin_.y = value->info.origin.position.y;

        receivedMap_=true;
    }

}

bool Perception::hasReceivedMap()
{
    return receivedMap_;
}

bool Perception::hasStartedMCL()
{
    return startedMCL_;
}