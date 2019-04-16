// Adapted from "Skeleton" by PHYLLIS FEI
// See: http://ima.nyu.sh/documentation/kinetic-interfaces-final-project-skeleton-phyllis/
// Added SVG support.
// TODO: readjust alignment of bones.

import KinectPV2.KJoint;
import KinectPV2.*;
KinectPV2 kinect;
KJoint[] joints;

BodyPart[] bones = new BodyPart[16];
int skeletonAge = 0;

PShape foot1, foot2, shank1, shank2, thigh1, thigh2;
PShape forearm1, forearm2, bigarm1, bigarm2, hand1, hand2;
PShape bodyUpper, bodyDown, head, neck;

PVector headR;
PVector neckR;
PVector shoulder1R, shoulder2R;
PVector elbow1R, elbow2R;
PVector wrist1R, wrist2R;
PVector hand1R, hand2R;
PVector spineBase, spineMid, spineShoulder;
PVector hip1R, hip2R;
PVector knee1R, knee2R;
PVector ankle1R, ankle2R;
PVector foot1R, foot2R;

float Attractionamount = 0;
int floorLevel = 300;

void setup() {
  fullScreen();
  background(255);
  setupKinect();
  loadImg();
  loadBones();
  int index = int( random(bones.length) );
  bones[index].isDetached = false;
}

void draw() {
  
  background(255);
  /*
  pushStyle();
  noStroke();
  fill(50);
  rect(0, height-floorLevel, width, floorLevel);
  fill(255);
  popStyle();*/


  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      joints = skeleton.getJoints();
      color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
      //awBody( joints );
    }
  }

  // get the body parts
  if (skeletonArray.size() == 0) {
    skeletonAge = 0;
    for (int i=0; i<bones.length; i++) {
      bones[i].isDetached = false;
      bones[i].vel = new PVector();
    }
  } else {
    skeletonAge++;
    updateVectorsFromSkeleton();
    PVector vector, pos;

    // ***** head
    vector = PVector.sub(headR, neckR);
    pos = new PVector(neckR.x, neckR.y);
    bones[0].updateFromSkeleton( vector, pos );
    bones[0].findBones(neckR);

    //// ***** bodyUpper
    vector = PVector.sub(spineMid, spineShoulder);
    pos = new PVector(spineShoulder.x, spineShoulder.y);
    bones[1].updateFromSkeleton( vector, pos );
    bones[1].findBones(spineShoulder);

    // ***** hand Left
    vector = PVector.sub(hand1R, wrist1R);
    pos = new PVector(wrist1R.x, wrist1R.y);
    bones[2].updateFromSkeleton( vector, pos );
    bones[2].findBones(wrist1R);

    // ***** hand Right
    vector = PVector.sub(hand2R, wrist2R);
    pos = new PVector(wrist2R.x, wrist2R.y);
    bones[3].updateFromSkeleton( vector, pos );
    bones[3].findBones(wrist2R);

    // ***** foot Left
    vector = PVector.sub(foot1R, ankle1R);
    pos = new PVector(ankle1R.x, ankle1R.y);
    bones[4].updateFromSkeleton( vector, pos );
    bones[4].findBones(ankle1R);

    // ***** foot Right
    vector = PVector.sub(foot2R, ankle2R);
    pos = new PVector(ankle2R.x, ankle2R.y);
    bones[5].updateFromSkeleton( vector, pos );
    bones[5].findBones(ankle2R);

    // ***** bigarm Left
    vector = PVector.sub(elbow1R, shoulder1R);
    pos = new PVector(shoulder1R.x, shoulder1R.y);
    bones[6].updateFromSkeleton( vector, pos );
    bones[6].findBones(shoulder1R);

    // ***** bigarm Right
    vector = PVector.sub(elbow2R, shoulder2R);
    pos = new PVector(shoulder2R.x, shoulder2R.y);
    bones[7].updateFromSkeleton( vector, pos );
    bones[7].findBones(shoulder2R);

    // ***** forearm Left
    vector = PVector.sub(wrist1R, elbow1R);
    pos = new PVector(elbow1R.x, elbow1R.y);
    bones[8].updateFromSkeleton( vector, pos );
    bones[8].findBones(elbow1R);

    // ***** forearm Right
    vector = PVector.sub(wrist2R, elbow2R);
    pos = new PVector(elbow2R.x, elbow2R.y);
    bones[9].updateFromSkeleton( vector, pos );
    bones[9].findBones(elbow2R);

    // ***** thigh Left
    vector = new PVector(knee1R.x - hip1R.x, knee1R.y - hip1R.y);
    pos = new PVector(hip1R.x, hip1R.y);
    bones[10].updateFromSkeleton( vector, pos );
    bones[10].findBones(hip1R);

    // ***** thigh Right
    vector = new PVector(knee2R.x - hip2R.x, knee2R.y - hip2R.y);
    pos = new PVector(hip2R.x, hip2R.y);
    bones[11].updateFromSkeleton( vector, pos );
    bones[11].findBones(hip2R);

    // ***** shank Left
    vector = PVector.sub(ankle1R, knee1R);
    pos = new PVector(knee1R.x, knee1R.y);
    bones[12].updateFromSkeleton( vector, pos );
    bones[12].findBones(knee1R);

    // ***** shank Right
    vector = PVector.sub(ankle2R, knee2R);
    pos = new PVector(knee2R.x, knee2R.y);
    bones[13].updateFromSkeleton( vector, pos );
    bones[13].findBones(knee2R);

    // ****** Neck
    vector = PVector.sub(spineShoulder, neckR);
    pos = new PVector(neckR.x, neckR.y);
    bones[14].updateFromSkeleton( vector, pos );
    bones[14].findBones(neckR);

    // *** BodyDown
    vector = PVector.sub(spineBase, spineMid);
    pos = new PVector(spineMid.x, spineMid.y);
    bones[15].updateFromSkeleton( vector, pos );
    bones[15].findBones(spineMid);

    // display
    for (int i=0; i<bones.length; i++) {
      BodyPart b = bones[i];
      PVector gravity = new PVector(0, 0.45);
      b.applyForce( gravity);
      b.updatePhysics();
      if (skeletonAge++ > 1600) {
        b.checkAcceleration();
      }
      b.checkBoundary();
      b.applyRestitution(-0.025);
      b.display();
      b.getPreviousPos();
    }
  }
  fill(0);
  text( skeletonAge, 10, 20 );
}


void loadImg() {
  foot1 = loadShape("foot1.svg");
  foot2 = loadShape("foot2.svg");
  shank1 = loadShape("shank1.svg");
  shank2 = loadShape("shank2.svg");
  thigh1 = loadShape("thigh1.svg");
  thigh2 = loadShape("thigh2.svg");
  forearm1 = loadShape("forearm1.svg");
  forearm2 = loadShape("forearm2.svg");
  bigarm1 = loadShape("bigarm1.svg");
  bigarm2 = loadShape("bigarm2.svg");
  hand1 = loadShape("hand1.svg");
  hand2 = loadShape("hand2.svg");
  bodyUpper = loadShape("bodyUpper.svg");
  head = loadShape("head.svg");
  neck = loadShape("neck.svg");
  bodyDown = loadShape("bodyDown.svg");
}
/*
void loadImg() {
  foot1 = loadImage("foot1.png");
  foot2 = loadImage("foot2.png");
  shank1 = loadImage("shank1.png");
  shank2 = loadImage("shank2.png");
  thigh1 = loadImage("thigh1.png");
  thigh2 = loadImage("thigh2.png");
  forearm1 = loadImage("forearm1.png");
  forearm2 = loadImage("forearm2.png");
  bigarm1 = loadImage("bigarm1.png");
  bigarm2 = loadImage("bigarm2.png");
  hand1 = loadImage("hand1.png");
  hand2 = loadImage("hand2.png");
  bodyUpper = loadImage("bodyUpper.png");
  head = loadImage("head.png");
  neck = loadImage("neck.png");
  bodyDown = loadImage("bodyDown.png");
}*/

void updateVectorsFromSkeleton() {
  headR = new PVector ( joints[KinectPV2.JointType_Head].getX(), joints[KinectPV2.JointType_Head].getY() );
  neckR = new PVector ( joints[KinectPV2.JointType_Neck].getX(), joints[KinectPV2.JointType_Neck].getY());
  shoulder1R = new PVector ( joints[KinectPV2.JointType_ShoulderLeft].getX(), joints[KinectPV2.JointType_ShoulderLeft].getY() );
  shoulder2R = new PVector ( joints[KinectPV2.JointType_ShoulderRight].getX(), joints[KinectPV2.JointType_ShoulderRight].getY() );
  elbow1R = new PVector ( joints[KinectPV2.JointType_ElbowLeft].getX(), joints[KinectPV2.JointType_ElbowLeft].getY() );
  elbow2R = new PVector ( joints[KinectPV2.JointType_ElbowRight].getX(), joints[KinectPV2.JointType_ElbowRight].getY() );
  wrist1R = new PVector ( joints[KinectPV2.JointType_WristLeft].getX(), joints[KinectPV2.JointType_WristLeft].getY() );
  wrist2R = new PVector ( joints[KinectPV2.JointType_WristRight].getX(), joints[KinectPV2.JointType_WristRight].getY() );
  hand1R = new PVector ( joints[KinectPV2.JointType_HandLeft].getX(), joints[KinectPV2.JointType_HandLeft].getY() );
  hand2R = new PVector ( joints[KinectPV2.JointType_HandRight].getX(), joints[KinectPV2.JointType_HandRight].getY() );
  spineBase = new PVector ( joints[KinectPV2.JointType_SpineBase].getX(), joints[KinectPV2.JointType_SpineBase].getY() );
  spineMid = new PVector  ( joints[KinectPV2.JointType_SpineMid].getX(), joints[KinectPV2.JointType_SpineMid].getY() );
  spineShoulder = new PVector  ( joints[KinectPV2.JointType_SpineShoulder].getX(), joints[KinectPV2.JointType_SpineShoulder].getY() );
  hip1R = new PVector ( joints[KinectPV2.JointType_HipLeft].getX(), joints[KinectPV2.JointType_HipLeft].getY() );
  hip2R = new PVector ( joints[KinectPV2.JointType_HipRight].getX(), joints[KinectPV2.JointType_HipRight].getY() );
  knee1R = new PVector ( joints[KinectPV2.JointType_KneeLeft].getX(), joints[KinectPV2.JointType_KneeLeft].getY() );
  knee2R = new PVector ( joints[KinectPV2.JointType_KneeRight].getX(), joints[KinectPV2.JointType_KneeRight].getY() );
  ankle1R =  new PVector ( joints[KinectPV2.JointType_AnkleLeft].getX(), joints[KinectPV2.JointType_AnkleLeft].getY() );
  ankle2R = new PVector ( joints[KinectPV2.JointType_AnkleRight].getX(), joints[KinectPV2.JointType_AnkleRight].getY() );
  foot1R = new PVector ( joints[KinectPV2.JointType_FootLeft].getX(), joints[KinectPV2.JointType_FootLeft].getY() );
  foot2R = new PVector ( joints[KinectPV2.JointType_FootRight].getX(), joints[KinectPV2.JointType_FootRight].getY() );
}

class BodyPart {
  String name;
  //PImage img;
  PShape img;
  PVector pos;
  PVector vel;
  PVector acc;
  float angle;
  float scale;
  float distance;
  boolean isDetached;
  PVector prePos;

  BodyPart(String _name, PShape _img) {
    name = _name;
    img = _img;
    angle = 0;
    pos = new PVector();
    prePos = new PVector();
    vel = new PVector();
    acc = new PVector();
    scale = 1.0;
    distance = 0;
    isDetached = false;
  }

  void checkAcceleration() {
    PVector vector = PVector.sub(pos, prePos);
    if (vector.mag() > 70) {
      println(name, vector.mag());
      isDetached = true;
      vector.mult(0.5);
      applyForce( vector );
      pushStyle();
      noStroke();
      fill(255, 0, 0, 100);
      ellipse(pos.x, pos.y, 100, 100);
      popStyle();
    }
  }

  void findBones(PVector skeletonVector) {
    if (isDetached) {  
      PVector vector = skeletonVector.copy().sub(pos);
      float distance = vector.mag();
      if (distance <= 20) {
        //vector.mult(0.03);
        isDetached = false;
      } else{
        vector.mult(0.01);
        applyForce(vector);
        //isDetached = false;
      }
      pushStyle();
      strokeWeight(1);
      stroke(100,30);
      line(skeletonVector.x, skeletonVector.y, pos.x, pos.y);
      popStyle();
    }
  }

  void getPreviousPos() {
    prePos.x = pos.x;
    prePos.y = pos.y;
  }

  void updatePhysics() {
    if ( isDetached ) {
      vel.add(acc);
      pos.add(vel);
      acc.mult(0);
    }
    vel.limit(50);
  }
  void applyForce( PVector force ) {
    if ( isDetached ) {
      PVector f = force.copy();
      acc.add( f );
      println(f);
    }
  }
  void updateFromSkeleton( PVector vector, PVector _pos ) {
    if ( !isDetached ) {
      pos = _pos.copy();
      PVector v = vector.copy();
      distance = v.mag();
      angle = v.heading();
    }
  }

  void display() {
    float adjustX = 0;
    float adjustY = 0;
    float adjustW = 2.0;
    float adjustH = 2.0;
    float adjustA = 0;

    if ( name.equals("head") ) {
       adjustY = -100; 
       adjustA = PI;
    } else if ( name.equals("bodyDown" ) ) {
    } else if ( name.equals("bigarm1") ) {
      adjustX = -80;
      adjustY = 0;
    } else if (name.equals("bigarm2" )) {
      adjustX = 80;
      adjustY = 0;
     } else if ( name.equals("forearm1") ) {
       adjustX = -80;
    } else if (name.equals("forearm2" )) {
      adjustX = 80;
    } else if ( name.equals("hand1") ) {
     adjustX = -80;
    } else if (name.equals("hand2" )) {
     adjustX = 80;
    } else if (name.equals("thigh1" )) {
    } else if (name.equals("thigh2" )) {
    } else if (name.equals("shank1" )) {
      adjustY = 100;
    } else if (name.equals("shank2" )) {
      adjustY = 100;
    } else if (name.equals("foot1" )) {
      adjustY = 200;
    } else if (name.equals("foot2" )) {
       adjustY = 100;
    } else if (name.equals("bodyUpper" )) {
    }
    
    scale = distance / img.height;
    pushStyle();
    pushMatrix();
    translate(pos.x, pos.y);
    
    rotate( angle + adjustA - PI/2 );
    shape(img, 
      -0.5*img.width * adjustW * scale + adjustX * scale, 
      adjustY * scale, 
      img.width*scale * adjustW, 
      img.height * scale * adjustH
     );
     
    popMatrix();
    popStyle();
  }

  void applyRestitution(float amount) {
    float value = 1.0 + amount;
    vel.mult( value );
  }

  void checkBoundary() {
    if (pos.x < 0) {
      pos.x = 0;
      vel.x *= -1;
    } else if (pos.x > width) {
      pos.x = width;
      vel.x *= -1;
    }
    if (pos.y < 0) {
      pos.y = 0;
      vel.y *= -1;
    } else if (pos.y > height - floorLevel) {
      pos.y = height - floorLevel;
      vel.y *= -1;
    }
  }
}

void setupKinect() {
  kinect = new KinectPV2(this);
  kinect.enableSkeletonColorMap(true);
  kinect.enableColorImg(true);
  kinect.init();
}

void loadBones(){
 bones[0] = new BodyPart("head", head);
  bones[1] = new BodyPart("bodyUpper", bodyUpper);
  bones[2] = new BodyPart("hand1", hand1);
  bones[3] = new BodyPart("hand2", hand2);
  bones[4] = new BodyPart("foot1", foot1);
  bones[5] = new BodyPart("foot2", foot2);
  bones[6] = new BodyPart("bigarm1", bigarm1);
  bones[7] = new BodyPart("bigarm2", bigarm2);
  bones[8] = new BodyPart("forearm1", forearm1);
  bones[9] = new BodyPart("forearm2", forearm2);
  bones[10] = new BodyPart("thigh1", thigh1);
  bones[11] = new BodyPart("thigh2", thigh2);
  bones[12] = new BodyPart("shank1", shank1);
  bones[13] = new BodyPart("shank2", shank2);
  bones[14] = new BodyPart("neck", neck);
  bones[15] = new BodyPart("bodyDown", bodyDown);
}
