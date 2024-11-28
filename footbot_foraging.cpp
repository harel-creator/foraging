/* Include the controller definition */
#include "footbot_foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#define COLLISION_THRESHOLD 0.1f  // Adjust based on simulation scale

/****************************************/
/****************************************/

CFootBotForaging::SFoodData::SFoodData() :
   HasFoodItem(false),
   FoodItemIdx(0),
   TotalFoodItems(0) {}

void CFootBotForaging::SFoodData::Reset() {
   HasFoodItem = false;
   FoodItemIdx = 0;
   TotalFoodItems = 0;
}

/****************************************/
/****************************************/

CFootBotForaging::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotForaging::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotForaging::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void CFootBotForaging::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
      GetNodeAttribute(t_node, "initial_explore_to_rest_prob", InitialExploreToRestProb);
      GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", FoodRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", FoodRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", CollisionRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "social_rule_rest_to_explore_delta_prob", SocialRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "social_rule_explore_to_rest_delta_prob", SocialRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
      GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootBotForaging::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = InitialRestToExploreProb;
   ExploreToRestProb = InitialExploreToRestProb;
   TimeExploringUnsuccessfully = 0;
   /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
   TimeRested = MinimumRestingTime;
   TimeSearchingForPlaceInNest = 0;

}

/****************************************/
/****************************************/

CFootBotForaging::CFootBotForaging() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL) {
   
}

CFootBotForaging::~CFootBotForaging() {
    //LogBehaviorStatistics();
    if (m_log_file.is_open()) {
        m_log_file.close();
    }
}

/****************************************/
/****************************************/

void CFootBotForaging::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   
   
   LOG << "Robot initialized with ID: " << GetId() << std::endl;
   // Create a unique log file for each robot
   std::string filename = "collision_log_" + GetId() + ".txt";
   m_log_file.open(filename, std::ios::out | std::ios::trunc);
   if (!m_log_file.is_open()) {
      LOGERR << "Failed to open log file for robot " << GetId() << std::endl;
   }

   

   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void CFootBotForaging::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         ReturnToNest();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::RED);
   /* Clear up the last exploration result */
   m_eLastExplorationResult = LAST_EXPLORATION_NONE;
   m_pcRABA->ClearData();
   m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);

   m_non_collision_start_tick = 0;
   this->m_collisionEnded = true; // There isn't a previous coillision
   ///reset the durations vectors
   this->m_last_behavior_durations.assign(NUMBER_OF_BEHAVIORS, 0);
   this->m_longest_non_collision_times.assign(NUMBER_OF_BEHAVIORS, 0);
   this->m_shortest_collision_durations.assign(NUMBER_OF_BEHAVIORS, std::numeric_limits<UInt64>::max());
   this->m_collision_number = 0;
   this->m_SBehaviorMetrics.resize(NUMBER_OF_BEHAVIORS);
   this->repel_vector = CVector2::ZERO;
}

/****************************************/
/****************************************/

void CFootBotForaging::UpdateState() {
   /* Reset state flags */
   m_sStateData.InNest = false;
   /* Read stuff from the ground sensor */
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
   /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray
    * area. 
    * The foot-bot has 4 sensors like this, two in the front
    * (corresponding to readings 0 and 1) and two in the back
    * (corresponding to reading 2 and 3).  Here we want the back sensors
    * (readings 2 and 3) to tell us whether we are on gray: if so, the
    * robot is completely in the nest, otherwise it's outside.
    */
   if(tGroundReads[2].Value > 0.25f &&
      tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f &&
      tGroundReads[3].Value < 0.75f) {
      m_sStateData.InNest = true;
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotForaging::CalculateVectorToLight() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

void CFootBotForaging::LogCollisionDetails(UInt64 collision_start_tick, UInt64 collision_end_tick, 
                         ECollisionBehavior current_behavior) {
    if (m_log_file.is_open()) {
        // Collision duration
        UInt64 collision_duration = collision_end_tick - collision_start_tick;
        if (collision_duration > MAX_COLLISION_DURATION_TICKS){
         LOG<<"ERROR in tick "<< collision_end_tick<< std::endl;
        }
        // Log details
        m_log_file << "Collision Start Tick: " << collision_start_tick << "\n";
        m_log_file << "Collision End Tick: " << collision_end_tick << "\n";
        m_log_file << "Collision Duration: " << collision_duration << " ticks\n";
        m_log_file << "Behavior: " << BehaviorToString(current_behavior) << "\n";
        m_log_file << "UCB Scores:\n";

        // Log UCB scores for all behaviors
        for (size_t i = 0; i < m_SBehaviorMetrics.size(); ++i) {
            const auto& metrics = m_SBehaviorMetrics[i];
            m_log_file << "  " << BehaviorToString(static_cast<ECollisionBehavior>(i)) << ": "
                            << std::fixed << std::setprecision(3)
                            << metrics.ucb_score << "\n";
        }

        m_log_file << "---------------------------------\n";
    }
}


std::string CFootBotForaging::BehaviorToString(ECollisionBehavior e) const {
    switch (e) {
        case TURN_180: return "TURN_180";
        case STOP: return "STOP";
        case DEFAULT: return "DEFAULT";
        case REPEL : return "REPEl";
        default: return "UNKNOWN";
    }
}

void CFootBotForaging::LogCollisionData(UInt64 collision_duration_ticks, UInt64 non_collision_duration_ticks, UInt64 collision_end_tick) {
    if (m_log_file.is_open()) {
        
        m_log_file << "Collision Summary:\n"
                   << "  Time since last collision (ticks): " << non_collision_duration_ticks << "\n"
                   << "  Collision start tick: " << (m_collision_start_tick) << "\n"
                   << "  Collision end tick: " << collision_end_tick << "\n"
                   << "  Collision duration (ticks): " << collision_duration_ticks << "\n"
                   << "  Chosen behavior: " << BehaviorToString(this->m_currentCollisionBehavior) << "\n\n";
    }
}

void CFootBotForaging::LogBehaviorStatistics() {
    if (m_log_file.is_open()) {
        m_log_file << "\n=== Behavior Statistics ===\n";
        for (size_t i = 0; i < NUMBER_OF_BEHAVIORS; ++i) {
            m_log_file << "Behavior: " << BehaviorToString(static_cast<ECollisionBehavior>(i)) << "\n";
            m_log_file << "  Longest Non-Collision Time: " << m_longest_non_collision_times[i] << " ticks\n";
            m_log_file << "  Shortest Collision Duration: " << m_shortest_collision_durations[i] << " ticks\n\n";
        }
    }
}
void CFootBotForaging::Destroy(){
   
}
void CFootBotForaging::ChooseRandomBehavior() {
    // Randomly select a behavior
    enum ECollisionBehavior number_of_behaviors = NUMBER_OF_BEHAVIORS;
    //m_pcRNG->Reset();
    int random_number = m_pcRNG->Uniform(argos::CRange<UInt32>(0, number_of_behaviors));
    m_currentCollisionBehavior = static_cast<ECollisionBehavior>(random_number);
    

    // Log the chosen behavior
    //LOG << "Robot " << m_robot_id << " chose behavior: " << BehaviorToString(m_currentCollisionBehavior) << std::endl;
}

void CFootBotForaging::ChooseBehaviorUsingUCB() {
    double exploration_factor = 1.0; // Adjust exploration factor as needed

    int best_behavior = 0;
    double highest_ucb_score = -std::numeric_limits<double>::max();

    for (size_t i = 0; i < m_SBehaviorMetrics.size(); ++i) {
        auto& metrics = m_SBehaviorMetrics[i];

        // Update UCB score for explored behaviors
        if (metrics.times_selected > 0) {
            double avg_reward = metrics.total_collision_time / metrics.times_selected;
            metrics.ucb_score = -avg_reward + exploration_factor * sqrt(log(static_cast<double>(m_collision_number)) / metrics.times_selected);
        }

        // Find the behavior with the highest UCB score
        if (metrics.ucb_score > highest_ucb_score) {
            highest_ucb_score = metrics.ucb_score;
            best_behavior = i;
        }
    }

    // Set the chosen behavior
    m_currentCollisionBehavior = static_cast<ECollisionBehavior>(best_behavior);
    m_currentCollisionBehavior = REPEL; //remove this, just for testing
}

CVector2 CFootBotForaging::BehaviorStop(){
   return CVector2(0.0f,0.0f);
}
CVector2 CFootBotForaging::BehaviorDefault(CVector2 cDiffusionVector){
   return -(cDiffusionVector.Normalize());
}
CVector2 CFootBotForaging::BehaviorRepel(CVector2 cDiffusionVector){
   //Its take the footboot about 22 ticks in this angle to make ~180 turn
   // He do so in the beggining of the collision and in the end of it
   if ((this->m_ticks_in_collisin < 22) || (this->m_ticks_in_collisin > (MAX_COLLISION_DURATION_TICKS -22))){
      return CVector2(1.0f,1.0f).Rotate(CRadians::PI);
   }
   // If the turn is over, move forward
   else{
      return CVector2(1.0f,0.0f);
   }   
   
}

CVector2 CFootBotForaging::BehaviorTurn(CVector2 cDiffusionVector){
   cDiffusionVector = CVector2::X;
   return -(cDiffusionVector.Rotate(CRadians(1.0)));
}

/****************************************/
CVector2 CFootBotForaging::handleCollision(CVector2 cDiffusionVector) {
   switch (this->m_currentCollisionBehavior) {
      case TURN_180: 
         //actally kindda turn left
         return BehaviorTurn(cDiffusionVector);
      case STOP:
         return BehaviorStop();
      case DEFAULT:
         return BehaviorDefault(cDiffusionVector);
      case REPEL:
         return BehaviorRepel(cDiffusionVector);
      default:
         return BehaviorDefault(cDiffusionVector);
   }
}



CVector2 CFootBotForaging::DiffusionVector(bool& b_collision) {
   // This section is to make sure the repel go to the end, 
   // and not just until there is a clear path
   if ((m_ticks_in_collisin < MAX_COLLISION_DURATION_TICKS)&&(m_currentCollisionBehavior == REPEL)){
      m_ticks_in_collisin ++;
      return BehaviorRepel(CVector2::ZERO);
   }
   

    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    CVector2 cDiffusionVector;
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }

    UInt64 current_tick = CSimulator::GetInstance().GetSpace().GetSimulationClock();
    bool was_in_collision = !m_collisionEnded;
    if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
       cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
        
        this->m_collisionEnded = true;
        b_collision = false;

         

        if (was_in_collision) {
            // Collision just ended

            // Add one to the specific collision counter
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].times_selected +=1;
            // Add the douretion of the collision just add so we can update the avg reward next time
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].total_collision_time += (current_tick - m_collision_start_tick);
            
            //Calculate the duration of the collision just ended
            UInt64 collision_duration_ticks = current_tick - m_collision_start_tick;
            
            // Update per-behavior stats
            m_last_behavior_durations[m_currentCollisionBehavior] = collision_duration_ticks;
            //check if the last collision was the fastest one with this behavior (for this robot)
            
            if (collision_duration_ticks < m_shortest_collision_durations[m_currentCollisionBehavior]) {
               //If it was, save it as the fastest
                m_shortest_collision_durations[m_currentCollisionBehavior] = collision_duration_ticks;
            }
            
            // Log data
            LogCollisionDetails(m_collision_start_tick, current_tick, m_currentCollisionBehavior);
            // Reset non-collision timer
            m_non_collision_start_tick = current_tick;
        }
        
        return CVector2::X;
    }
    else {
      //This section happen when there is a collision
      
        b_collision = true;
        if (!was_in_collision) {
            // Collision just started
            
            // Add one to the total collision counter
            this->m_collision_number ++;

            // The time of free travel, just statistics
            UInt64 non_collision_duration_ticks = current_tick - m_non_collision_start_tick;
            if (non_collision_duration_ticks > m_longest_non_collision_times[m_currentCollisionBehavior]) {
                m_longest_non_collision_times[m_currentCollisionBehavior] = non_collision_duration_ticks;
            }
            m_last_non_collision_duration = current_tick - m_non_collision_start_tick;

            m_collision_start_tick = current_tick; // Start collision timer
            m_collisionEnded = false;
            
            //ChooseRandomBehavior();
            ChooseBehaviorUsingUCB();
        }

         //Check if the collision taking too long. If it is, change the behavior
         UInt64 collision_duration = current_tick - m_collision_start_tick; //need this line
         m_ticks_in_collisin = collision_duration;
         // Check if the collision has exceeded the maximum allowed duration
         if (collision_duration > MAX_COLLISION_DURATION_TICKS) {
            this->m_collision_number ++;
         // Log the timeout event
            LOG << "[Robot " << GetId() << "] Collision timeout reached. Switching behavior.\n";
            LogCollisionDetails(m_collision_start_tick, current_tick-1, m_currentCollisionBehavior);

            // Collision-ending updates
            // Add one to the specific collision counter
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].times_selected +=1;
            // Add the douretion of the collision just add so we can update the avg reward next time
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].total_collision_time += MAX_COLLISION_DURATION_TICKS;
            
            // Re-choose behavior using UCB
            ChooseBehaviorUsingUCB();

            // Reset collision start tick for the new behavior
            m_collision_start_tick = current_tick;
         }
      return handleCollision(cDiffusionVector);
    }
}

void collision_start_setting(){

}

void CFootBotForaging::collision_ending_updates(UInt64 collision_duration){
   // Add one to the specific collision counter
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].times_selected +=1;
            // Add the douretion of the collision just add so we can update the avg reward next time
            this->m_SBehaviorMetrics[static_cast<int>(this->m_currentCollisionBehavior)].total_collision_time += MAX_COLLISION_DURATION_TICKS;
}




/****************************************//*
CVector2 CFootBotForaging::DiffusionVector(bool& b_collision) {
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }

   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      m_collisionEnded = true;
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      return -(cDiffusionVector.Normalize());
      //return handleCollision(cDiffusionVector);
   }
}*/


/****************************************/
/****************************************/

void CFootBotForaging::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CFootBotForaging::Rest() {
   /* If we have stayed here enough, probabilistically switch to
    * 'exploring' */
   if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
      m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.RestToExploreProb) {
      m_pcLEDs->SetAllColors(CColor::GREEN);
      m_sStateData.State = SStateData::STATE_EXPLORING;
      m_sStateData.TimeRested = 0;
   }
   else {
      ++m_sStateData.TimeRested;
      /* Be sure not to send the last exploration result multiple times */
      if(m_sStateData.TimeRested == 1) {
         m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
      }
      /*
       * Social rule: listen to what other people have found and modify
       * probabilities accordingly
       */
      const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
      for(size_t i = 0; i < tPackets.size(); ++i) {
         switch(tPackets[i].Data[0]) {
            case LAST_EXPLORATION_SUCCESSFUL: {
               m_sStateData.RestToExploreProb += m_sStateData.SocialRuleRestToExploreDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
               m_sStateData.ExploreToRestProb -= m_sStateData.SocialRuleExploreToRestDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
               break;
            }
            case LAST_EXPLORATION_UNSUCCESSFUL: {
               m_sStateData.ExploreToRestProb += m_sStateData.SocialRuleExploreToRestDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
               m_sStateData.RestToExploreProb -= m_sStateData.SocialRuleRestToExploreDeltaProb;
               m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
               break;
            }
         }
      }
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::Explore() {
   /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */
   bool bReturnToNest(false);
   /*
    * Test the first condition: have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    */
   if(m_sFoodData.HasFoodItem) {
      /* Apply the food rule, decreasing ExploreToRestProb and increasing
       * RestToExploreProb */
      m_sStateData.ExploreToRestProb -= m_sStateData.FoodRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      /* Store the result of the expedition */
      m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
      /* Switch to 'return to nest' */
      bReturnToNest = true;
   }
   /* Test the second condition: we probabilistically switch to 'return to
    * nest' if we have been wandering for some time and found nothing */
   else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
      if (m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.ExploreToRestProb) {
         /* Store the result of the expedition */
         m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
         /* Switch to 'return to nest' */
         bReturnToNest = true;
      }
      else {
         /* Apply the food rule, increasing ExploreToRestProb and
          * decreasing RestToExploreProb */
         m_sStateData.ExploreToRestProb += m_sStateData.FoodRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
   }
   /* So, do we return to the nest now? */
   if(bReturnToNest) {
      /* Yes, we do! */
      m_sStateData.TimeExploringUnsuccessfully = 0;
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_pcLEDs->SetAllColors(CColor::BLUE);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   }
   else {
      /* No, perform the actual exploration */
      ++m_sStateData.TimeExploringUnsuccessfully;
      UpdateState();
      /* Get the diffusion vector to perform obstacle avoidance */
      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision);
      /* Apply the collision rule, if a collision avoidance happened */
      if(bCollision) {
         /* Collision avoidance happened, increase ExploreToRestProb and
          * decrease RestToExploreProb */
         m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
         m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
         m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
      }
      /*
       * If we are in the nest, we combine antiphototaxis with obstacle
       * avoidance
       * Outside the nest, we just use the diffusion vector
       */
      if(m_sStateData.InNest) {
         /*
          * The vector returned by CalculateVectorToLight() points to
          * the light. Thus, the minus sign is because we want to go away
          * from the light.
          */
         SetWheelSpeedsFromVector(
            m_sWheelTurningParams.MaxSpeed * cDiffusion -
            m_sWheelTurningParams.MaxSpeed * 0.25f * CalculateVectorToLight());
      }
      else {
         /* Use the diffusion vector only */
         SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::ReturnToNest() {
   /* As soon as you get to the nest, switch to 'resting' */
   UpdateState();
   /* Are we in the nest? */
   if(m_sStateData.InNest) {
      /* Have we looked for a place long enough? */
      if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
         /* Yes, stop the wheels... */
         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
         /* Tell people about the last exploration attempt */
         m_pcRABA->SetData(0, m_eLastExplorationResult);
         /* ... and switch to state 'resting' */
         m_pcLEDs->SetAllColors(CColor::RED);
         m_sStateData.State = SStateData::STATE_RESTING;
         m_sStateData.TimeSearchingForPlaceInNest = 0;
         m_eLastExplorationResult = LAST_EXPLORATION_NONE;
         return;
      }
      else {
         /* No, keep looking */
         ++m_sStateData.TimeSearchingForPlaceInNest;
      }
   }
   else {
      /* Still outside the nest */
      m_sStateData.TimeSearchingForPlaceInNest = 0;
   }
   /* Keep going */
   bool bCollision;
   SetWheelSpeedsFromVector(
      m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
      m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotForaging, "footbot_foraging_controller")
