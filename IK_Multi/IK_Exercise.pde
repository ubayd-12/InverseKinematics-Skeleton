//Based off the example code from the class "IK-Exercise" by Stephen Guy

Vec2 root;

//Endeffectors
Vec2 endEffector1;
Vec2 endEffector2;


Vec2 rootMovementDirection = new Vec2(1, 0); //Which direction we moving the root x,y
float rootMovementSpeed = 1.0;
int lastDirectionChangeTime = 0;
int upperLimit = 50;
int horizontalMargin = 50;
Vec2 initialRootPosition;

//Class that defines the segments for the arm
class ArmSegment {
  Vec2 start;
  Vec2 end;
  float length;
  float angle;

  ArmSegment(Vec2 start, float length, float angle) {
    this.start = new Vec2(start.x, start.y);
    this.length = length;
    this.angle = angle;
    this.end = new Vec2(0, 0);
    this.updateEnd();
  }

  void updateEnd() {
    Vec2 offset = polarToCartesian(this.angle, this.length);
    this.end.x = this.start.x + offset.x;
    this.end.y = this.start.y + offset.y;
  }
}

void setup(){
  size(600,600);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");

  //Setup the skeleton
  root = new Vec2(300,450);
  randomGoal = new Vec2(100, 100);
  base1 = new ArmSegment(new Vec2(300, 450), 50, 0.3);
  base2 = new ArmSegment(base1.end, 50, 0.3);

  redArm1 = new ArmSegment(base2.end, 100, 0.3);
  redArm2 = new ArmSegment(redArm1.end, 50, 0.3);
  redArm3 = new ArmSegment(redArm2.end, 25, 0.3);

  greenArm1 = new ArmSegment(base2.end, 100, 0.3);
  greenArm2 = new ArmSegment(greenArm1.end, 50, 0.3);
  greenArm3 = new ArmSegment(greenArm2.end, 25, 0.3);
  initialRootPosition = new Vec2(root.x, root.y);
}

ArmSegment base1, base2;
ArmSegment greenArm1, greenArm2, greenArm3;
ArmSegment redArm1, redArm2, redArm3;

Vec2 randomGoal;
Vec2 mouseTarget = new Vec2(mouseX, mouseY);
Vec2 fixedRandGoal = randomGoal;

int lastUpdateTime = 0;

// CCD Angle Adjustment:
void updateAngle(Vec2 targetPosition, Vec2 jointStartPosition, Vec2 jointEndPosition, float[] currentAngle, float maxJointAngle, float minJointAngle) {
  Vec2 directionToTarget = targetPosition.minus(jointStartPosition);
  Vec2 directionToEndEffector = jointEndPosition.minus(jointStartPosition);

  Vec2 normalizedDirectionToTarget = directionToTarget.normalized();
  Vec2 normalizedDirectionToEndEffector = directionToEndEffector.normalized();

  float normalizedDotProduct = dot(normalizedDirectionToTarget, normalizedDirectionToEndEffector);
  normalizedDotProduct = clamp(normalizedDotProduct, -1, 1);

  float computedAngleDifference = acos(normalizedDotProduct);

  if (cross(directionToTarget, directionToEndEffector) < 0) {
      currentAngle[0] += computedAngleDifference;
  } else {
      currentAngle[0] -= computedAngleDifference;
  }
  currentAngle[0] = constrain(currentAngle[0], minJointAngle, maxJointAngle);
}

// endEffector back to base calculation
void solve() { 

  float[] jointAngle = new float[1];

  //set both goals initally
  mouseTarget = new Vec2(mouseX, mouseY);
  fixedRandGoal = randomGoal;

  //endeffector to base

  jointAngle[0] = redArm3.angle;
  updateAngle(fixedRandGoal, redArm3.start, endEffector2, jointAngle, 1, -1);
  redArm3.angle = jointAngle[0];
  fk();


  jointAngle[0] = redArm2.angle;
  updateAngle(fixedRandGoal, redArm2.start, endEffector2, jointAngle, 0.75, -0.75);
  redArm2.angle = jointAngle[0];
  fk();


  jointAngle[0] = redArm1.angle;
  updateAngle(fixedRandGoal, base2.start, endEffector2, jointAngle, 0.5, -0.5);
  redArm1.angle = jointAngle[0];
  fk();


  jointAngle[0] = greenArm3.angle;
  updateAngle(mouseTarget, greenArm3.start, endEffector1, jointAngle, 0.75, -0.75);
  greenArm3.angle = jointAngle[0];
  fk();


  jointAngle[0] = greenArm2.angle;
  updateAngle(mouseTarget, greenArm2.start, endEffector1, jointAngle, 0.5, -0.5);
  greenArm2.angle = jointAngle[0];
  fk();


  jointAngle[0] = greenArm1.angle;
  updateAngle(mouseTarget, base2.start, endEffector1, jointAngle, 1, -1);
  greenArm1.angle = jointAngle[0];
  fk();


  if (base1.start.distanceTo(mouseTarget) < base1.start.distanceTo(fixedRandGoal)) {
      jointAngle[0] = base2.angle;
      updateAngle(mouseTarget, base1.start, endEffector1, jointAngle, 1, -1);
  } else {
      jointAngle[0] = base2.angle;
      updateAngle(fixedRandGoal, base1.start, endEffector2, jointAngle, 1, -1);
  }
  
  base2.angle = jointAngle[0];
  fk();


  jointAngle[0] = base1.angle;
  updateAngle(mouseTarget, root, endEffector2, jointAngle, -1, -2);
  base1.angle = jointAngle[0];
  fk();
}

void fk() {
  float angleSum;

  angleSum = base1.angle;
  base1.start = calculateJointPosition(root, angleSum, base1.length);

  angleSum += base2.angle;
  base2.start = calculateJointPosition(base1.start, angleSum, base2.length);

  angleSum += greenArm1.angle;
  greenArm2.start = calculateJointPosition(base2.start, angleSum, greenArm1.length);

  angleSum += greenArm2.angle;
  greenArm3.start = calculateJointPosition(greenArm2.start, angleSum, greenArm2.length);

  endEffector1 = calculateJointPosition(greenArm3.start, angleSum + greenArm3.angle, greenArm3.length);

  angleSum = base1.angle + base2.angle + redArm1.angle;
  redArm2.start = calculateJointPosition(base2.start, angleSum, redArm1.length);

  angleSum += redArm2.angle;
  redArm3.start = calculateJointPosition(redArm2.start, angleSum, redArm2.length);

  endEffector2 = calculateJointPosition(redArm3.start, angleSum + redArm3.angle, redArm3.length);
}

Vec2 calculateJointPosition(Vec2 start, float angleSum, float length) {
    return new Vec2(cos(angleSum) * length, sin(angleSum) * length).plus(start);
}

Vec2 obstacleCenter = new Vec2(400, 300);
float obstacleRadius = 25;

float newLinkLength = 100;
float newLinkCircleRadius = 30;

void updateRootPosition() {
  float newX = root.x + rootMovementDirection.x * rootMovementSpeed;
  float newY = root.y + rootMovementDirection.y * rootMovementSpeed;

  root.x = constrain(newX, horizontalMargin, width - horizontalMargin);

  if (newY < initialRootPosition.y - upperLimit) {
      newY = initialRootPosition.y - upperLimit;
      changeDirection();
  }
  if (newY > initialRootPosition.y) {
      newY = initialRootPosition.y;
  }
  root.y = newY;

  if (root.x == horizontalMargin || root.x == width - horizontalMargin) {
      changeDirection();
  }
}

void changeDirection() {
    float angle = random(TWO_PI);
    rootMovementDirection.x = cos(angle);
    rootMovementDirection.y = sin(angle);
}

//Draw functions

void drawNewLinkAndCircle() {
  
  Vec2 newLinkEnd = new Vec2(cos(base1.angle + base2.angle) * newLinkLength, sin(base1.angle + base2.angle) * newLinkLength).plus(base1.start);
  
  fill(255, 228, 196);
  noStroke();
  ellipse(newLinkEnd.x, newLinkEnd.y, newLinkCircleRadius * 2, newLinkCircleRadius * 2);
}



void drawLegs() {
  float legLength = 150;
  float legAngle = PI / 6;

  Vec2 leftLegEnd = new Vec2(root.x - sin(legAngle) * legLength, root.y + cos(legAngle) * legLength);
  Vec2 rightLegEnd = new Vec2(root.x + sin(legAngle) * legLength, root.y + cos(legAngle) * legLength);

  stroke(0);
  strokeWeight(10);

  line(root.x, root.y, leftLegEnd.x, leftLegEnd.y);

  line(root.x, root.y, rightLegEnd.x, rightLegEnd.y);
}

void drawObstacle(Vec2 obstacle, float radius) {
  ellipse(obstacle.x, obstacle.y, radius * 2, radius * 2);
}

void updateKinematics() {
  fk();
  solve();
}

void updateTargetPosition() {
  if (millis() - lastUpdateTime > 5000) {
      randomGoal.x = constrain(random(width), 10, width - 10);
      randomGoal.y = constrain(random(height), 10, height - 10);
      lastUpdateTime = millis();
  }
}

void drawBackground() {
  background(250, 250, 250);
}

void drawRobot() {
  drawLink(root, base1.angle, base1.length, 214, 168, 133);
  drawLink(base1.start, base1.angle + base2.angle, base2.length, 100, 100, 255);

  // Green arm
  drawLink(base2.start, base1.angle + base2.angle + greenArm1.angle, greenArm1.length, 0, 255, 0);
  drawLink(greenArm2.start, base1.angle + base2.angle + greenArm1.angle + greenArm2.angle, greenArm2.length, 0, 200, 0);
  drawLink(greenArm3.start, base1.angle + base2.angle + greenArm1.angle + greenArm2.angle + greenArm3.angle, greenArm3.length, 255, 215, 0); // End effector

  // Red arm
  drawLink(base2.start, base1.angle + base2.angle + redArm1.angle, redArm1.length, 255, 0, 0);
  drawLink(redArm2.start, base1.angle + base2.angle + redArm1.angle + redArm2.angle, redArm2.length, 200, 0, 0);
  drawLink(redArm3.start, base1.angle + base2.angle + redArm1.angle + redArm2.angle + redArm3.angle, redArm3.length, 0, 191, 255); // End effector
}

void drawLink(Vec2 position, float angle, float length, int r, int g, int b) {
  fill(r, g, b);
  pushMatrix();
  translate(position.x, position.y);
  rotate(angle);
  rect(0, -15, length, 15);
  popMatrix();
}

void drawTargets() {
  fill(0, 255, 0);
  circle(randomGoal.x, randomGoal.y, 10);
  drawLegs();
  drawNewLinkAndCircle();
  fill(255, 0, 0);
  noStroke();
  ellipse(mouseX, mouseY, 10, 10);
}

void draw() {
   updateRootPosition();
  updateKinematics();
  updateTargetPosition();
  drawBackground();
  drawRobot();
  drawTargets();
}

Vec2 polarToCartesian(float angle, float length) {
  return new Vec2(cos(angle) * length, sin(angle) * length);
}
