//Based off off IK_Excercise by Stephen Guy

void setup(){
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//Root
Vec2 root = new Vec2(0,0);

//Upper Arm
float l0 = 100; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 100;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 100;
float a2 = 0.3; //Wrist joint

Vec2 circleCenter = new Vec2(100, 150);
float circleRadius = 20;

Vec2 start_l1,start_l2,endPoint;

boolean armCircleCol(Vec2 circleCenter, float circleRadius) {

    if (segCircleCol(root, start_l1, circleCenter, circleRadius)) {
        return true;
    }

    if (segCircleCol(start_l1, start_l2, circleCenter, circleRadius)) {
        return true;
    }

    if (segCircleCol(start_l2, endPoint, circleCenter, circleRadius)) {
        return true;
    }

    return false;
}

boolean segCircleCol(Vec2 segmentStart, Vec2 segmentEnd, Vec2 circleCenter, float circleRadius) {
  
    Vec2 closestPoint = closestPointOnSegment(segmentStart, segmentEnd, circleCenter);
    float distanceToCircle = closestPoint.distanceTo(circleCenter);

    return distanceToCircle <= circleRadius;
}

Vec2 closestPointOnSegment(Vec2 segmentStart, Vec2 segmentEnd, Vec2 point) {
    Vec2 segmentVector = segmentEnd.minus(segmentStart);
    float segmentLengthSquared = segmentVector.x * segmentVector.x + segmentVector.y * segmentVector.y;

    if (segmentLengthSquared == 0.0) {
        return segmentStart;
    }

    float t = dot(point.minus(segmentStart), segmentVector) / segmentLengthSquared;
    t = clamp(t, 0.0, 1.0);

    Vec2 projection = segmentStart.plus(segmentVector.times(t));
    return projection;
}


void solve() {
    Vec2 goal = new Vec2(mouseX, mouseY);
    float angleDiff, old_a2, old_a1, old_a0;
    int maxIterations = 100;
    int iterations = 0;

    float adjustmentFactor = 0.01;

    do {
        System.out.println("test");
        old_a0 = a0;
        old_a1 = a1;
        old_a2 = a2;

  
        Vec2 startToGoal = goal.minus(root);
        if (startToGoal.length() < .0001) return;
        Vec2 startToEndEffector = endPoint.minus(root);
        float dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
        dotProd = clamp(dotProd, -1, 1);
        angleDiff = acos(dotProd);
        if (cross(startToGoal, startToEndEffector) < 0)
            a0 += angleDiff;
        else
            a0 -= angleDiff;
        fk();

        startToGoal = goal.minus(start_l1);
        startToEndEffector = endPoint.minus(start_l1);
        dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
        dotProd = clamp(dotProd, -1, 1);
        angleDiff = acos(dotProd);
        if (cross(startToGoal, startToEndEffector) < 0)
            a1 += angleDiff;
        else
            a1 -= angleDiff;
        fk();

        startToGoal = goal.minus(start_l2);
        startToEndEffector = endPoint.minus(start_l2);
        dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
        dotProd = clamp(dotProd, -1, 1);
        angleDiff = acos(dotProd);
        if (cross(startToGoal, startToEndEffector) < 0)
            a2 += angleDiff;
        else
            a2 -= angleDiff;
        fk();

        if (!armCircleCol(circleCenter, circleRadius)) {
            break;
        }
    
        a0 = old_a0 + (random(-1, 1) * adjustmentFactor);
        a1 = old_a1 + (random(-1, 1) * adjustmentFactor);
        a2 = old_a2 + (random(-1, 1) * adjustmentFactor);
    
        fk();
       
        println("Adjusting angles - a0: " + a0 + ", a1: " + a1 + ", a2: " + a2);
    
        iterations++;
        if (iterations >= maxIterations) {
            println("Maximum iterations reached without resolving collision");
            break;
        }

        if (armCircleCol(circleCenter, circleRadius)) {
            a2 = old_a2;
            angleDiff *= 0.5;
            if (cross(startToGoal, startToEndEffector) < 0)
                a2 += angleDiff;
            else
                a2 -= angleDiff;
            fk();
        } else {
            break;
        }
        if (iterations >= maxIterations) {
          System.out.println("Maximum iterations reached without resolving collision");
          break;
        }
    } while (true);

    println("Angle 0:", a0, "Angle 1:", a1, "Angle 2:", a2);
}


void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  endPoint = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  

  fill(200,0,180);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  fill(255, 0, 0);
  noStroke();
  ellipse(circleCenter.x, circleCenter.y, (circleRadius * 2) / 2, (circleRadius * 2) /2); 
  
}
