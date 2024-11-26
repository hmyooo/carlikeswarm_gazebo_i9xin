void Ped::Tagent::computeForces(turtlebot turtlebot) {
  // update neighbors
  // NOTE - have a config value for the neighbor range
  const double neighborhoodRange = 10.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);
//（desired force）：智能体想要到达的目标点产生的力.
//social force）：考虑到环境中其他智能体和turtlebot的位置，计算智能体间的社交距离保持力，使其避免与其他智能体或turtlebot过于接近。
//obstacle force）：避免与障碍物碰撞的排斥力 
//当其他智能体或turtlebot靠近时，社交力和障碍力会增大，从而影响到智能体的行进方向和速度，以避免碰撞。但如果社交力、障碍力不够强或者邻居检测范围不足，可能导致智能体间的碰撞现象
// update forces
  desiredforce = desiredForce();
  // turtlebot.request();
  if (forceFactorSocial > 0) socialforce = socialForce(turtlebot.pos_list,turtlebot.vec_list);
  if (forceFactorObstacle > 0) obstacleforce = obstacleForce();
  myforce = myForce(desiredDirection);
}


Ped::Tvector Ped::Tagent::desiredForce() {
  // get destination
  Twaypoint* waypoint = getCurrentWaypoint();

  // if there is no destination, don't move
  if (waypoint == NULL) {
    desiredDirection = Ped::Tvector();
    Tvector antiMove = -v / relaxationTime;
    return antiMove;
  }

  // compute force
  Tvector force = waypoint->getForce(*this, &desiredDirection);

  return force;
}


Ped::Tvector Ped::Twaypoint::getForce(const Ped::Tagent& agent,
                                      Ped::Tvector* desiredDirectionOut,
                                      bool* reachedOut) const {
  // set default output parameters
  if (reachedOut != NULL) *reachedOut = false;
  if (desiredDirectionOut != NULL) *desiredDirectionOut = Ped::Tvector();

  Ped::Tvector agentPos = agent.getPosition();
  Ped::Tvector destination = closestPoint(agentPos, reachedOut);
  Ped::Tvector diff = destination - agentPos;

  Ped::Tvector desiredDirection = diff.normalized();  // 向量归一化
  Tvector force = (desiredDirection * agent.getVmax() - agent.getVelocity()) /
                  agent.getRelaxationTime();

  if (desiredDirectionOut != NULL) *desiredDirectionOut = desiredDirection;

  return force;
}

Ped::Tvector Ped::Tagent::obstacleForce() const {
  // obstacle which is closest only
  Ped::Tvector minDiff;
  double minDistanceSquared = INFINITY;

  for (const Tobstacle* obstacle : scene->obstacles) {
    Ped::Tvector closestPoint = obstacle->closestPoint(p);
    Ped::Tvector diff = p - closestPoint;
    double distanceSquared = diff.lengthSquared();  // use squared distance to
    // avoid computing square
    // root
    if (distanceSquared < minDistanceSquared) {
      minDistanceSquared = distanceSquared;
      minDiff = diff;
    }
  }

  double distance = sqrt(minDistanceSquared) - agentRadius;
  double forceAmount = exp(-distance / forceSigmaObstacle);
  return forceAmount * minDiff.normalized();
}

Ped::Tvector Ped::Tagent::socialForce(vector<Tvector> &pos_list,vector<Tvector> &vec_list) const {
  // define relative importance of position vs velocity vector
  // (set according to Moussaid-Helbing 2009)
  Tvector diff;
  Tvector velDiff;
  Tvector robot; 
  Tvector force;
  // while(!turtlebot.pos_list.empty()){

  //     if(){}

  // }
  const double lambdaImportance = 2.0;//位置和速度向量之间的相对重要性权重

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double gamma = 0.35;//定义速度相互作用的模型参数

  // define speed interaction
  // (set according to Moussaid-Helbing 2009)
  const double n = 2;

  // define angular interaction
  // (set according to Moussaid-Helbing 2009)
  const double n_prime = 3;//定义角度相互作用的模型参数

  // define eye contact interaction: added by xzt
  const double eye_sigma = 0.2;
  const double eye_prob = 0.8;
if(!pos_list.empty()){
  for(int i=0;i<pos_list.size();i++){
      if(pos_list[i].x<(p.x+6)&&pos_list[i].y<(p.y+6)&&pos_list[i].x>(p.x-6)
      &&pos_list[i].y>(p.y-6)){
    diff=pos_list[i]-p;
    velDiff=v-vec_list[i];
    Tvector diffDirection = diff.normalized();
        // Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
        Tvector interactionVector = lambdaImportance * velDiff+diffDirection;//相互作用方向
    double interactionLength = interactionVector.length();//计算相互作用长度interactionLength和相互作用方向interactionDirection
    Tvector interactionDirection = interactionVector / interactionLength;

    // compute angle theta (between interaction and position difference vector)
    Ped::Tangle theta = interactionDirection.angleTo(diffDirection);//相互作用方向与位置差异向量之间的夹角

    // compute model parameter B = gamma * ||D||
    double B = gamma * interactionLength;

    double thetaRad = theta.toRadian();
    double forceVelocityAmount =
        -exp(-diff.length() / B -
             (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
    double forceAngleAmount =
        -theta.sign() *
        exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

    Tvector forceVelocity = forceVelocityAmount * interactionDirection;
    Tvector forceAngle =
        forceAngleAmount * diffDirection.leftNormalVector();
//模型参数和角度计算速度相关的力forceVelocity和角度相关的力forceAngle
    force += 2*forceVelocity + 3*forceAngle;
  }
    }

      pos_list.clear();
      vec_list.clear();

  }
 
  for (const Ped::Tagent* other : neighbors) {
    // don't compute social force to yourself
    if (other->id == id) continue;

    // compute difference between both agents' positions
    diff = other->p - p;
    // change this can let robot not collide with pedestrian: by xzt
    if(other->getType() == ROBOT) diff /= robotPosDiffScalingFactor;
      
    Tvector diffDirection = diff.normalized();

    // compute difference between both agents' velocity vectors
    // Note: the agent-other-order changed here
    velDiff = v - other->v;

    // compute interaction direction t_ij
    Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
    double interactionLength = interactionVector.length();
    Tvector interactionDirection = interactionVector / interactionLength;

    // compute angle theta (between interaction and position difference vector)
    Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

    // compute model parameter B = gamma * ||D||
    double B = gamma * interactionLength;

    double thetaRad = theta.toRadian();
    double forceVelocityAmount =
        -exp(-diff.length() / B -
             (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
    double forceAngleAmount =
        -theta.sign() *
        exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

    Tvector forceVelocity = forceVelocityAmount * interactionDirection;
    Tvector forceAngle =
        forceAngleAmount * interactionDirection.leftNormalVector();

    force += forceVelocity + forceAngle;
    

  }

  return force;
}