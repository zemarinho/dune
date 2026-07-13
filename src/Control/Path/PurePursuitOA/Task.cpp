//***************************************************************************
// Copyright 2007-2025 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Jose Marinho                                                     *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>
#include <vector>
#include <deque>
#include <sstream>
#include <unordered_map>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace PurePursuitOA
    {
      using DUNE_NAMESPACES;

      //! GOALS:

      //! - através de p da tarefa definir circulos e poligonos de 4 lados como áreas de exclusão
      //! - algoritmo que detete se o sistema está dentro da ára
      //! - algoritmo que permita cumprir o path sem entrar nas áreas de exclusão
      //! -

      // -> é garantido que o utilizador não define áreas sobrepostas e que o espaço entre as áreas é o suficiente para não atrapalhas o caminho
      // -> o veiculo está sempre à superficie

      //! formato circulo: lat-lon-radius (degrees, degrees, meters)
      //! formator poligono 4 lados: lat_p0-lon_p0-width-height

      //!           *-----------*
      //!           |           |
      //!           |           |
      //!    p0 ->  *-----------*

      //! ex: 41.18279098/-8.70796953/56;41.18186403/-8.70552352/175/160

      //Par de valores de longitude/latitude em coordenadas geográficas (usado para posições ou distâncias)
      struct Position_G
      {
        double lon;
        double lat;
      };

      //Par de valores em coordenadas cartesianas (em metros) (usado para distâncias ou velocidades)
      struct Position_C
      {
        double h;
        double v;
      };
      struct Obstacle
      {
        char type = 'E';                      //P=point, Z=zone, E=not defined -> error
        char shape = 'E';                     //C=circle, R=rectangle, E=not defined -> error
        double safetyZoneDistance = 0;        //meters; distance between perimether and safety zone margin
        std::deque<Position_G> positions;  //list of the las positions of the obstacle (standard = 5)
        double radius = 0;                    //meters
        double width = 0;                     //meters
        double height = 0;                    //meters
        std::string id = "";                  //id
      };



      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredHeading m_heading;                  //direção para onde está a ir
        IMC::DesiredPath m_path;                        //ponto para o qual está a ir
        std::string m_obs_msg;                          //mensagem com os obstáculos
        std::string m_add_obst;                         //mensagem com obstáculo extra para adicionar
        std::string m_rmv_obst;                         //mensagem com obstáculo para remover
        std::vector<Obstacle> m_obstacles;              //vetor com os obstáculos
        std::unordered_map<std::string, int> m_obst_index;      //índices de cada obstáculo no vetor de obstáculos
        Position_G finalPos;                              //destino imediato
        Position_G endPoint;                              //próximo GoTo do percurso
        Position_G currPos;                               //posição atual
        int oldAvoidState = 0;
        int currAvoidState = 0;
        bool clear_obst_list = false;

        bool epIsSet = false;                                     //se o endPoint já foi definido
        bool canChangeEP = true;                                  //pode-se alterar o endPoint
        Position_G oldPath = {m_path.end_lon, m_path.end_lat};      //verifica seo m_path.end mudou ou não
        Position_G newPath;                                         //verifica seo m_path.end mudou ou não

        bool finalPosChanged = true;
        bool hasStoredOriginal = false;           //se em endPoint está guardada o GoTo correto

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx)
        {
          param("Obstacle Message", m_obs_msg)
          .defaultValue("")
          .description("Obstacles definition.");

          param("Obstacle Add", m_add_obst)
          .defaultValue("")
          .description("Add an obstacle to the list.");

          param("Obstacle Rmv", m_rmv_obst)
          .defaultValue("")
          .description("Remove an obstacle from the list.");

          bind<IMC::PlanControlState>(this);
          // bind<IMC::VehicleState>(this);
        }

        void
        consume(const IMC::PlanControlState* pcs)
        {
          if (pcs->getSource() != getSystemId())
            return;

          //war("Received PlanControlState: %s | maneuver: %s", pcs->plan_id.c_str(), pcs->man_id.c_str());
        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();
          if (paramChanged(m_obs_msg))
          {
            clear_obst_list = true;
            processMessage(m_obs_msg);
          }
          if (paramChanged(m_add_obst))
          {
            processMessage(m_add_obst);
          }
          if (paramChanged(m_rmv_obst))
          {
            removeObstacle(m_rmv_obst);
          }
        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }

        void
        onPathActivation(void)
        {
          // Activate heading controller.
          enableControlLoops(IMC::CL_YAW);
        }

        void
        onPathDeactivation(void)
        {
          // Deactivate heading controller.
          disableControlLoops(IMC::CL_YAW);
        }

        /**
         * @brief Calculates the center os the rectangle based on bottom left corner and sides dimensions
         * @param obstacle
         */
        Position_G
        centerLonLat(std::vector<std::string>& dimensions) //calcula o centro do obstáculo
        {
          /*find the center of the rectangle*/
          Position_G pos;
          pos.lon = stod(dimensions[0]) + horDist2lonDist(stod(dimensions[2]), stod(dimensions[1]))/2;
          pos.lat = stod(dimensions[1]) + verDist2latDist(stod(dimensions[3]))/2;
          return pos;
        }

        /**
         * @brief Converts horizontal distance in meters to longitude distance in degrees
         * @param horDist   distance in meters to be converted in degrees
         * @param latitude  latitude from where we want to convert the distance
         */
        double
        horDist2lonDist(double horDist, double latitude)
        {
          return horDist * 180 / (6371000*cos(latitude*M_PI/180)*M_PI);
        }

        /**
         * @brief Converts vertical distance in meters to latitude distance in degrees
         * @param verDist   distance in meters to be converted in degrees
         */
        double
        verDist2latDist(double verDist)
        {
          return verDist * 180 / (6371000*M_PI);
        }

        /**
         * @brief Converts longitude distance in degrees to horizontal distance in meters
         * @param lonDist   distance in degrees to be converted in meters
         * @param latitude  latitude from where we want to convert the distance
         */
        double
        lonDist2horDist(double lonDist, double latitude)
        {
          //war("%f, %f", lonDist, latitude);
          return 6371000*cos(Angles::radians(latitude))*Angles::radians(lonDist);
        }

        /**
         * @brief Converts latitude distance in degrees to vertizontal distance in meters
         * @param latDist   distance in degrees to be converted in meters
         */
        double
        latDist2verDist(double latDist)
        {
          return 6371000*Angles::radians(latDist);
        }

        /**
          * @brief Initializes m_path with standart values relatives to the current state
          * @param curr_lat current latitude
          * @param curr_lon current longitude
          * @param ts       current tracking state
          */
        void pathInit(double curr_lat, double curr_lon, const TrackingState& ts)
        {
          m_path.start_lat = curr_lat;                      //guardar posição atual como inicial
          m_path.start_lon = curr_lon;                      //guardar posição atual como inicial

          m_path.end_lon = Angles::degrees(ts.lon_en);      //guardar o destino original
          m_path.end_lat = Angles::degrees(ts.lat_en);      //guardar o destino original

          m_path.speed = 1;
          m_path.end_z = ts.end.z;
          m_path.end_z_units = ts.end.z_units;
        }

        /**
         * @brief  Splits the message with obstacles, isolates their values and stores each of them on m_obstacles
         * @param  msg message where the obstacles are defined
         * @note   Message strucure: Type,Shape safetyZoneDistance centerLon,centerLat,R/bottomLeftLon,bottomLeftLat,horizontalDist,verticalDist id
         * @note   Message example:  ZC 10 -8.70796953,41.18279098,56 11 & ZR 10 -8.70552352,41.18186403,175,160 13
         * @note   '&' separates obstacles
         * @note   ' ' separates fields of an obstacle
         * @note   ',' separates values of a field
         * @note   '.' decimal divider
         *
         * */
        void
        processMessage(std::string msg)
        {
          // war("EEEEEEEEEEEEEEEEEEEEEE");
          std::vector<std::string> obstacles_str;
          String::split(msg, "&", obstacles_str);
          Position_G centro;
          std::string id;
          double largura;
          double altura;
          bool existeID;


          if (clear_obst_list)
          {
            m_obstacles.clear();
            m_obst_index.clear();
            clear_obst_list = false;
          }


          for (const auto& obs_str : obstacles_str)     //percorrer a mensagem com os obstáculos
          {
            std::vector<std::string> fields;
            String::split(obs_str, " ", fields);
            existeID = false;
            Obstacle obstacle;

            if (fields.size() != 4)                     //verificar se tem o número certo de campos
            {
              war("Incorrectly defined obstacle: %ld fields instead of 4", fields.size());
              continue;
            }

            if (fields[3] == "")                        //verificar se tem id
            {
              war(">> Parameter incorrectly defined: without id");
              continue;
            }
            obstacle.id = fields[3];

            if (m_obst_index.find(obstacle.id) != m_obst_index.end())      //verifica se o id já existe
            {
              obstacle.positions = m_obstacles[m_obst_index[obstacle.id]].positions;

            }

            std::string obstacleParams = fields[0];

            if (obstacleParams.size() != 2)             //verificar se campo de tipo tem o tamanho correto
            {
              war("Parameter incorrectly defined: Params wrong size -> %li", obstacleParams.size());
              continue;
            }

            if ((obstacleParams[0] != 'P' && obstacleParams[0] != 'Z') ||
                (obstacleParams[1] != 'C' && obstacleParams[1] != 'R'))       //verificar se os valores do campo de tipo são válidos
            {
              war("Parameter incorrectly defined: Value without meaning");
              continue;
            }

            obstacle.type  = obstacleParams[0];
            obstacle.shape = (obstacle.type == 'P') ? 'C':obstacleParams[1];            //se o obstáculo é um ponto é tratado como um círculo

            std::vector<std::string> dimensions_str;
            String::split(fields[2], ",", dimensions_str);

            if (obstacle.shape == 'C' && dimensions_str.size() != 3)             //verificar se a forma do obstáculo é consistente com o número de parâmetros passado
            {

              war(">> Parameter incorrectly defined: incorrect number of location parameters (1): %li; expected 3", dimensions_str.size());
              if (dimensions_str.size() != 4)
              {
                continue;
              }
              war(">> Obstacle identified as a rectangle, treated as a circle, height ignored, width treated as radius");
            }
            if (obstacle.shape == 'R' && dimensions_str.size() != 4)
            {
              war(">> Parameter incorrectly defined: incorrect number of location parameters (2): %li; expected 4", dimensions_str.size());
              continue;
            }

            centro = {std::stod(dimensions_str[0]), std::stod(dimensions_str[1])};
            if (obstacle.shape == 'R')                                            //se o obstáculo é retangular, calcular o centro a partir do canto inferior esquerdo
            {
              centro = centerLonLat(dimensions_str);
            }

            if (m_obst_index.find(obstacle.id) != m_obst_index.end())               //se o obstáculo já existir, modificar apenas a lista de posições
            {
              war(">> Obstacle already exists: %s", obstacle.id.c_str());
              war(">> Obstacle overwritten!!!"); //NOTA: modificar quando for feita receção para AIS ou outro sistema para não estar constantemente a ser imprimida a mensagem
              obstacle.positions.push_front(centro);
              obstacle.positions.pop_back();
              m_obstacles[m_obst_index[obstacle.id]] = obstacle;
              continue;
            }

            for (int i=0; i<5; i++)                                               //colocar todas as posiçoes iguais num novo obstáculo
            {
              obstacle.positions.push_front(centro);
            }

            obstacle.safetyZoneDistance = std::stod(fields[1]);

            if (obstacle.type == 'P')                                             //se o obstáculo é um ponto a distância que interessa é a margem de segurança
            {
              obstacle.radius = 0;
              m_obst_index[obstacle.id] = m_obstacles.size();
              m_obstacles.push_back(obstacle);
              war(">> Obstacle added successfully: %s", obstacle.id.c_str());
              continue;
            }

            if (obstacle.shape == 'C')                                //guardar dimensões do obstáculo
            {
              obstacle.radius = std::stod(dimensions_str[2]);
              m_obst_index[obstacle.id] = m_obstacles.size();
              m_obstacles.push_back(obstacle);
              war(">> Obstacle added successfully: %s", obstacle.id.c_str());
            }

            obstacle.width = std::stod(dimensions_str[2]);
            obstacle.height = std::stod(dimensions_str[3]);

            m_obst_index[obstacle.id] = m_obstacles.size();
            m_obstacles.push_back(obstacle);
            war(">> Obstacle added successfully: %s", obstacle.id.c_str());
          }
        }

        /**
         * @brief Removes an obstacle from the list
         * @param id id of the obstacle to be removed
         */
        void
        removeObstacle(std::string id)
        {
          auto it = m_obst_index.find(id);

          if (it == m_obst_index.end())
          {
            war("Obstacle doesn't exist");
          }
          else
          {
            int pos = it->second;
            int lastPos = m_obstacles.size() - 1;

            if(pos != lastPos)
            {
              m_obstacles[pos] = m_obstacles[lastPos];
              m_obst_index[m_obstacles[pos].id] = pos;
            }

            m_obstacles.pop_back();
            m_obst_index.erase(it);

            war("Obstacle successefully removed: %s", id.c_str());
          }
        }

        /**
         * @brief Verifies if the vehicle is near an obstacle and calls the respective function acording the obstacle shape
         * @param obstacles list of obstacles to be checked
         * @return 1 if avoiding something, 0 if don't
         */
        int
        checkPosition(const std::vector<Obstacle> obstacles)
        {
          int avoidingcollision = 0;
          // int counter = 0;
          for (const auto &obstacle : obstacles)
          {
            double horizontalDistanceV, horizontalDistanceO; //referência é o veículo, referência é o obstáculo
            double verticalDistanceV, verticalDistanceO; //referência é o veículo, referência é o obstáculo
            double veicleObstacleDistance;

            horizontalDistanceV = lonDist2horDist(obstacle.positions.front().lon - currPos.lon, currPos.lat);
            horizontalDistanceO = -horizontalDistanceV;

            verticalDistanceV = latDist2verDist(obstacle.positions.front().lon - currPos.lat);
            verticalDistanceO = -verticalDistanceV;

            veicleObstacleDistance = sqrt(horizontalDistanceV*horizontalDistanceV+verticalDistanceV*verticalDistanceV);

            // war("Obstacle %d @ %fm", counter++, veicleObstacleDistance);
            // inf("I'm Here: %f %f", currPos.lon, currPos.lat);

            if (obstacle.shape == 'C') //caso de obstáculo ter forma circular (ponto ou zona)
            {
              if (obstacle.type == 'P') //caso de obstáculo ser ponto
              {
                if (veicleObstacleDistance < obstacle.safetyZoneDistance)
                {
                  inf("Trying to avoid collision Static Circle Point");
                  goAroundCircle(obstacle, horizontalDistanceV, verticalDistanceV);
                  avoidingcollision = 1;
                }
                else
                {
                  // DO NOTHING
                }
              }
              else if (obstacle.type == 'Z') //caso de obstáculo ser zona
              {
                //inf("It's a static circle zone");
                if (veicleObstacleDistance < obstacle.radius + obstacle.safetyZoneDistance)
                {
                  inf("Trying to avoid collision Static Circle Zone");
                  goAroundCircle(obstacle, horizontalDistanceV, verticalDistanceV);
                  avoidingcollision = 1;
                }
                else
                {
                  //  DO NOTHING
                }
              }
            }
            else if (obstacle.shape == 'R') //caso de obstáculo ter forma retangular
            {
              if (obstacle.type == 'P') //caso de o obstáculo ser um ponto;  Points defined with sqare shape are treated as circles
              {
                if (veicleObstacleDistance < obstacle.safetyZoneDistance)
                {
                  inf("Trying to avoid collision Static Rectangle Point");
                  goAroundCircle(obstacle, horizontalDistanceV, verticalDistanceV);
                  avoidingcollision = 1;
                }
                else
                {
                  // DO NOTHING
                }
              }
              else if (obstacle.type == 'Z') //caso de o obstáculo ser uma zona
              {
                if ((horizontalDistanceO < obstacle.width/2 + obstacle.safetyZoneDistance) &&
                    (horizontalDistanceO > 0 - obstacle.width/2 - obstacle.safetyZoneDistance) &&
                    (verticalDistanceO < obstacle.height/2 + obstacle.safetyZoneDistance) &&
                    (verticalDistanceO > 0 - obstacle.height/2 - obstacle.safetyZoneDistance)) //dentro da zona proibida
                {
                  inf("Trying to avoid collision Static Rectangle Poin");
                  goAroundRectangle(obstacle, horizontalDistanceO, verticalDistanceO);
                  avoidingcollision = 1;
                }
                else
                {
                  // DO NOTHING
                }
              }
            }

            // inf("Going to:        %f %f", m_path.end_lon, m_path.end_lat);
            // inf("Avoiding collision: %d", avoidingcollision);
          }

          if (!avoidingcollision)
          {
            m_heading.value = Angles::normalizeRadian(atan2(latDist2verDist(finalPos.lat - currPos.lat), lonDist2horDist(finalPos.lon - currPos.lon, currPos.lat)));

            m_path.end_lon = finalPos.lon;
            m_path.end_lat = finalPos.lat;
          }

          return avoidingcollision;
        }

        int lado = -1; // +1 esquerda, -1 direita
        /**
         * @brief Calculates an intermidiate Goto for going around a circle and passes it to m_path
         * @param obstacle obstacle in the way
         * @param veicObstHorDist horizontal distance of the obstacle from the vehicle
         * @param veicObstVerDist vertizontal distance of the obstacle from the vehicle
         */
        void
        goAroundCircle(Obstacle obstacle, double veicObstHorDist, double veicObstVerDist)
        {
          inf("Avoiding collision Static Circle");

          double obstRadPosition; //posição angular do centro do obstáculo relativamente ao veículo -pi:pi
          double destRadPosition; //posição angular do próximo GoTo relativamente ao veículo -pi:pi
          double nowHeading;
          double shift = 0.35 * (obstacle.radius+obstacle.safetyZoneDistance);
          double horShift, verShift;  //meters
          double lonShift, latShift;  //degrees
          double angular_distance;    //radians

          obstRadPosition = Angles::normalizeRadian(atan2(veicObstVerDist, veicObstHorDist));
          destRadPosition = Angles::normalizeRadian(atan2(latDist2verDist(endPoint.lat-currPos.lat), lonDist2horDist(endPoint.lon-currPos.lon, currPos.lat)));
          angular_distance = Angles::normalizeRadian(destRadPosition - obstRadPosition);

          // inf("Obstacle Direction: %f", obstRadPosition/M_PI);
          // inf("Shift: %f", shift);
          // war("m_headiung.value: %f", m_heading.value);
          // inf("Angular Distance: %f", Angles::degrees(angular_distance));

          if (angular_distance > 0)
          {
            nowHeading = Angles::normalizeRadian(obstRadPosition + (M_PI/2));
          }
          else
          {
            nowHeading = Angles::normalizeRadian(obstRadPosition - (M_PI/2));
          }

          //inf("Heading Direction: %f", nowHeading*180/M_PI);
          horShift = shift * cos(Angles::normalizeRadian(nowHeading));
          verShift = shift * sin(Angles::normalizeRadian(nowHeading));

          // inf("Horizontal Shift: %f", horShift);
          // inf("Vertical Shift: %f", verShift);
          // inf("Lon/Lat center obstacle: %f %f", obstacle.obstacle.positions.lon, obstacle.positions.lat);
          lonShift = horDist2lonDist(horShift, obstacle.positions.front().lat);
          latShift = verDist2latDist(verShift);

          m_path.end_lon = currPos.lon + lonShift;
          m_path.end_lat = currPos.lat + latShift;
          // m_heading.value = nowHeading;
          // inf("m_path circle:   %f %f", m_path.end_lon, m_path.end_lat);

        }

        /**
         * @brief Calculates an intermidiate Goto for going around a rectangle and passes it to m_path
         * @param obstacle obstacle in the way
         * @param veicObstHorDist horizontal distance of the obstacle from the vehicle
         * @param veicObstVerDist vertizontal distance of the obstacle from the vehicle
         *
         * @code
         *    ---------------------------
         *    |   .  SAFETYZONE ---->   |
         *    | A -------------------...|
         *    | | |    OBSTACLE     |   |
         *    |   |                 | | |
         *    |...------------------- V |
         *    |       <-------      .   |
         *    ---------------------------
         * @endcode
         */
        void
        goAroundRectangle(Obstacle obstacle, double veicObstHorDist, double veicObstVerDist)
        {
          war("Avoiding collision Static Rectangle");
          // inf("largura:   %f", obstacle.width);
          // inf("altura:    %f", obstacle.height);
          // inf("HorDist:   %f", veicObstHorDist);
          // inf("VerDist:   %f", veicObstVerDist);
          // inf("SafeDist:  %f", obstacle.safetyZoneDistance);

          if ((veicObstHorDist < 0 - obstacle.width/2) &&
              (veicObstVerDist > 0 - obstacle.height/2))      //à esquerda do obstáculo
          {
            m_path.end_lon = currPos.lon;
            m_path.end_lat = currPos.lat + verDist2latDist(obstacle.height/2 - veicObstVerDist + obstacle.safetyZoneDistance);


            // war("AAAAAAAA");

            m_heading.value = M_PI/2;
          }
          else if ((veicObstHorDist > 0 + obstacle.width/2) &&
                   (veicObstVerDist < 0 + obstacle.height/2))      //à direita do obstáculo
          {
            m_path.end_lon = currPos.lon;
            m_path.end_lat = currPos.lat - verDist2latDist(obstacle.height/2 + veicObstVerDist + obstacle.safetyZoneDistance);

            // war("BBBBBBBBB");
            m_heading.value = -M_PI/2;
          }
          else if ( (veicObstVerDist > 0 + obstacle.height/2) &&
                    (veicObstHorDist > 0 - obstacle.width/2))    //acima do obstáculo
          {
            m_path.end_lat = currPos.lat;
            m_path.end_lon = currPos.lon + horDist2lonDist(obstacle.width/2 - veicObstHorDist + obstacle.safetyZoneDistance, obstacle.positions.front().lat);

            // war("CCCCCCCC");

            m_heading.value = M_PI;
          }
          else if ( (veicObstVerDist < 0 - obstacle.height/2) &&
                    (veicObstHorDist < 0 + obstacle.width/2))    //abaixo do obstáculo
          {
            m_path.end_lat = currPos.lat;
            m_path.end_lon = currPos.lon - horDist2lonDist(obstacle.width/2 + veicObstHorDist + obstacle.safetyZoneDistance, obstacle.positions.front().lat);

            // war("DDDDDDDDD");
            m_heading.value = 0;
          }
        }

        /* void
        onDesiredPath(const IMC::DesiredPath* dp)
        {
          PathController::onDesiredPath(dp);
          // war("DesiredPath: %f %f %d", dp->end_lon, dp->end_lat, dp->getSourceEntity());
          // war("m_path:      %f %f %d", m_path.end_lon, m_path.end_lat, m_path.getSourceEntity());
          // war("self:  %u", getEntityId());

          // if (dp->getSourceEntity() == getEntityId())
          // {

          // }
          return;
        } */

        /**
         * @brief vreifica se dois obstáculos pontuais estão em rota de colisão
         */
        void
        colisionCourse(Obstacle obstacle)
        {
          /**
           *TODO: adicionar velocidade e heading à mensagem de obstáculo
           *TODO: arranjarvariável com velocidade atual do veículo
           *TODO: avaliar e implementar algoritmo fixado no chatgtp gmail 1
           *TODO: colisionCourse() vai ser chamada na condição de o obstáculo ser pontual
           *TODO: acrescentar if no checkPosition para verificar se a posição do obstáculo se alterou, e só nesse caso chamar colisionCourse()
           */

          Position_G veicDiference_G;
          veicDiference_G.lon = finalPos.lon - currPos.lon;
          veicDiference_G.lat = finalPos.lat - currPos.lat;

          //conversão para metros;
          Position_C veicDiference_C;
          veicDiference_C.h = lonDist2horDist(veicDiference_G.lon, veicDiference_G.lat);
          veicDiference_C.v = latDist2verDist(veicDiference_G.lat);

          double veicDistance = sqrt(veicDiference_C.h * veicDiference_C.h + veicDiference_C.v * veicDiference_C.v);

          Position_C veicDiferenceNormalized = {veicDiference.h/veicDistance, veicDiference.v/veicDistance};

          //!arranjar valor da velocidade atual do veículo
          Position_C veicVelocityVector = {veicDiferenceNormalized.h * veicVelocity, veicDiferenceNormalized.v * veicVelocity};


        }




        int clic = 0;
        int contador = 1;
        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          clic++;
          war("Step----------------------------------------");
          double curr_lat = state.lat;
          double curr_lon = state.lon;
          WGS84::displace(state.x, state.y, &curr_lat, &curr_lon);
          bool dsptch = false;

          finalPos = {Angles::degrees(ts.lon_en), Angles::degrees(ts.lat_en)};

          inf("Next Pos:        %f %f", finalPos.lon, finalPos.lat);
          war("Final Pos:       %f %f", endPoint.lon, endPoint.lat);

          currPos.lon = Angles::degrees(curr_lon);          //posição atual do veículo em coordenadas geográficas para fazer
          currPos.lat = Angles::degrees(curr_lat);          //os cálculos com posição do obstáculo definidas em coordendas geográficas

          if (contador)
          {
            pathInit(curr_lat, curr_lon, ts);
            // processMessage(m_obs_msg);
            contador--;
          }

          // inf("m_path:          %f %f", m_path.end_lon, m_path.end_lat);
          // inf("speed:           %f", ts.speed);

          // inf("%s", ((ts.nearby) ? "Nearby" : "Not Nearby"));

          inf("I'm Here:        %f %f", currPos.lon, currPos.lat);

          for (auto obst : m_obstacles)
          {
            inf ("%s", obst.id.c_str());
          }
          newPath = {Angles::degrees(m_ts.lon_en), Angles::degrees(m_ts.lat_en)};

          // inf("%d", getEntityId());
          // inf("%d", m_path.getSourceEntity());
          // inf("TrackingStat:    %f %f", newPath.lon, newPath.lat);
          // inf("Center:          %f %f", m_obstacles[1].obstacle.positions.lon, m_obstacles[1].obstacle.positions.lat);


          currAvoidState = checkPosition(m_obstacles);

          lado = (!currAvoidState) ? 0 : lado;

          if (!epIsSet && currAvoidState && !oldAvoidState)   //se o DesiredPath foi definido por este controlador e o endPoint ainda não foi definido
          {
            // war("AAAAAAAAAAAAAAAAAA");
            endPoint = {oldPath.lon, oldPath.lat};
            epIsSet = true;
          }

          // Se DEIXOU de evitar colisão, restaurar o destino original
          if (!currAvoidState && oldAvoidState && epIsSet)
          {
            war("Saiu de evasão, restaurando destino original");
            m_path.end_lon = endPoint.lon;      //restaurar o GoTo original
            m_path.end_lat = endPoint.lat;      //restaurar o GoTo original
            epIsSet = false;                    //o endPoint pode ser atualizado para o próximo GoTo
            dsptch = true;
          }

          if(newPath.lon != oldPath.lon || newPath.lat != oldPath.lat)    //verificar se o m_path.end mudou
          {
            // war("BBBBBBBBBBBBBBBBBB");

            finalPos = {newPath.lon, newPath.lat};                                           //se tiver mudado, atualizar o destino imediato
            finalPosChanged = true;
          }
          oldPath = newPath;

          if (!currAvoidState && m_ts.nearby && (clic % 20 == 0))
          {
            // war("CCCCCCCCCCCC");
            // Nota: Se já restaurou na lógica acima, isto não vai executar novamente
            if (epIsSet)
            {
              m_path.end_lon = endPoint.lon;      //passar o próximo GoTo
              m_path.end_lat = endPoint.lat;      //passar o próximo GoTo
              epIsSet = false;                    //o endPoint pode ser atualizado para o próximo GoTo
              m_ts.nearby = false;                //já não está próximo do destino imediato
              dsptch = true;
            }
          }

          if (ts.cc)
          {
            m_heading.value = Angles::normalizeRadian(m_heading.value + state.psi - ts.course);
          }

          if ( currAvoidState != oldAvoidState || dsptch)           // Se o estado de avoidance atual mudou
          {
            inf("Mudou de estado");

            // inf("m_path 2:        %f %f", m_path.end_lon, m_path.end_lat);
            m_path.end_lon = Angles::normalizeRadian(Angles::radians(m_path.end_lon));
            m_path.end_lat = Angles::normalizeRadian(Angles::radians(m_path.end_lat));

            dispatch(m_path, DF_LOOP_BACK);               // Avisar que mudamos o DesiredPath

            m_path.end_lon = Angles::degrees(m_path.end_lon);
            m_path.end_lat = Angles::degrees(m_path.end_lat);

            // setEndPoint(&m_path);                         // Alterar de facto o DesiredPath
            dsptch = false;

          }

          oldAvoidState = currAvoidState;

          //war("lat %f lon %f", curr_lat, curr_lon);

          m_heading.value = ts.los_angle;
          dispatch(m_heading);
        }
      };
    }
  }
}

DUNE_TASK
