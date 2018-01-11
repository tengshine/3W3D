var stringAngle = 22;
var bowP1 = 6.35, bowP2 = 6.47;
var tempP1 = bowP1,
  tempP2 = bowP2;

let isSlur = (type) => {
  if (type && type[0] === 'S')
    return true;
  else
    return false;
}

let isDownBow = (type) => { // including D, DS, DL, ...
  return type[0] === 'D';
}

let isStaccato = (type) => { // including portato
  return type[1] === 'S' || type[1] === 'P';
}

let isPortato = (type) => {
  return type[1] === 'P';
}

let isLift = (type) => { // lifted rest (L), or lifted note (XL)
  if (type && (type[0] === 'L' || type[1] === 'L'))
    return true;
  else
    return false;
}

let isLiftRequired = (event) => {
  // default: not lifted ...
  // including prevEvent.type != nextEvent.type
  if (isLift(event.type))
    return true;
  else
    return false;
}
var anglei = 0,
  anglej = 0;

function makeHuman(x,y,z){
  human = new THREE.Object3D();

  var head = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 32), new THREE.MeshLambertMaterial({
    color: 0x666666
  }));
  head.position.set(0, 7, 0);
  var nose = new THREE.Mesh(new THREE.ConeGeometry(0.25, 1, 32), new THREE.MeshLambertMaterial({
    color: 0x666666
  }));
  head.add(nose);
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, -0.2, 1);
  human.add(head);
  head.rotation.y = Math.PI / 5;

  body = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 6, 32), new THREE.MeshLambertMaterial({
    color: 0x666666
  }));
  body.position.set(0, 3, 0);
  human.add(body);

  var rArm1 = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.25, 3, 32), new THREE.MeshLambertMaterial({
    color: 0xffffff
  }));
  var rArm2 = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.25, 3, 32), new THREE.MeshLambertMaterial({
    color: 0x666666
  }));

  link1 = makeLink(UPPERARM_LEN);
  link1.position.set(-1.15, 6, 0);

  scene.add(link1);
  link2 = makeLink(FOREARM_LEN);
  link1.add(link2);
  link2.position.set(3, 0, 0);

  human.add(link1);

  var LeftArm1 = new THREE.Object3D();
  LeftArm1.rotation.x = -Math.PI / 3.1;
  LeftArm1.rotation.y = Math.PI / 18.5;
  LeftArm1.rotation.z = Math.PI / 10;
  LeftArm1.position.set(1.15, 6, 0);
  var lArm1 = rArm1.clone();
  LeftArm1.add(lArm1);

  lArm1.position.set(0, -1.5, 0);

  var LeftArm2 = new THREE.Object3D();
  LeftArm2.position.set(0, -3, 0);
  LeftArm2.rotation.x = -Math.PI / 3;

  var lArm2 = rArm2.clone();
  LeftArm2.add(lArm2);
  LeftArm1.add(LeftArm2);
  lArm2.position.set(0, -1.5, 0);

  human.add(LeftArm1);
  human.position.set(x,y,z);
  scene.add(human);
}

function p1top2() {
  if (tempP1 > bowP1) anglei = -0.01;
  if (tempP1 < bowP1) anglei = 0.01;
  if (tempP2 > bowP2) anglej = -0.01;
  if (tempP2 < bowP2) anglej = 0.01;
}

var CCDsys = function() {

  this.thetas = [];
  this.axes = [];

  this.target = new THREE.Vector3();

  // FK function set elsewhere
  this.setFK = function(fkEval) {

    this.fk = fkEval;

  }
  this.CCD_axis = function(axis, id, angleLo, angleHi) {

    this.axis = axis.clone();
    this.jointid = id;
    var thetaLo = angleLo === undefined ? -1e4 : angleLo; // default: no limits
    var thetaHi = angleHi === undefined ? 1e4 : angleHi;
    this.limits = new THREE.Vector2(thetaLo, thetaHi);

  };

  this.update = function() {

    var end = new THREE.Vector3();
    var base = new THREE.Vector3();

    var theta = this.thetas;
    var axes = this.axes;
    var target = this.target;


    var njoints = axes[axes.length - 1].jointid + 1;
    //console.log ('njoints: ' + njoints);
    var joints = [];
    for (var i = 0; i <= njoints; i++) joints[i] = new THREE.Vector3();

    this.fk(theta, joints);
    end.copy(joints[joints.length - 1]);

    // convergence
    var eps = 1e-1;
    var MAXITER = 20;

    var t_target = new THREE.Vector3();
    var t_end = new THREE.Vector3();
    var tmpV = new THREE.Vector3();


    for (var iter = 0; iter < MAXITER; iter++) {
      for (var i = axes.length - 1; i >= 0; i--) {
        base.copy(joints[axes[i].jointid]);


        var axis = axes[i].axis.clone();
        for (var j = i - 1; j >= 0; j--)
          axis.applyMatrix4(new THREE.Matrix4().makeRotationAxis(axes[j].axis, theta[j]));


        tmpV.subVectors(target, base);
        tmpV = proj2plane(tmpV, axis);
        t_target.copy(tmpV.normalize());

        tmpV.subVectors(end, base);
        tmpV = proj2plane(tmpV, axis);
        t_end.copy(tmpV.normalize());

        var dotV = t_end.dot(t_target);
        var angle = Math.acos(CLAMP(dotV, -1, 1));
        tmpV.crossVectors(t_end, t_target);
        var sign = (tmpV.dot(axis) > 0) ? 1 : -1;
        theta[i] += sign * angle;
        if (startClick) axes[i].limits.x = -1e4;
        theta[i] = CLAMP(theta[i], axes[i].limits.x, axes[i].limits.y)

        this.fk(theta, joints);
        end.copy(joints[joints.length - 1]);

        if (end.distanceTo(target) < eps) {
          return 1;
        }
      }

    }

    if (iter < MAXITER)
      return 1;
    else {
      //console.log("do not converge");
      return 0;
    }
  }

  function proj2plane(p, n) {
    return p.clone().projectOnPlane(n);
  }

  function CLAMP(x, xlo, xhi) {
    if (x < xlo)
      return xlo;
    if (x > xhi)
      return xhi;
    return x;
  }
}
function dynamic2Length(dynamic, bowCenter, duration) {

  // 0. extra long stroke ... return 1.0

  // 1. consider duration (2 beats <--> 1.0)

  var len = Math.min(duration / 2, 1.0);

  // 2. consider dynamics
  len = len * dynamic;

  console.log('dy:' + dynamic + ' len: ' + len);

  return len;

}
function makeSequence(eventArray) {

  var keyframes = [];
  var lastKeyStop;


  // count-in standby
  var keyframe0 = {};
  keyframe0.key = -4;
  keyframe0.pos = 0.5; // standby at midbow
  keyframes.push(keyframe0);

  // SET UP based on first event
  event = eventArray[0];
  var keyframe0 = {};
  keyframe0.key = event.beat - 1;
  keyframe0.pos = 0.5;
  keyframes.push(keyframe0);
  var keyframe1 = {};
  keyframe1.key = event.beat;
  keyframe1.pos = isDownBow(event.type) ? event.pos - event.length / 2 : event.pos + event.length / 2;
  keyframes.push(keyframe1);
  lastKeyStop = keyframe1.pos;

  // slur states
  // first three: cumulative
  var eventsInSlur = []; // events in this stroke (including the first event)
  var slurDur = 0; // time
  var slurLength; // motion length
  var slurType, slurStart; // set by first event in this stroke;

  for (var i = 0; i < eventArray.length; i++) {

    var event = eventArray[i];
    var prevEvent = eventArray[i - 1];
    var nextEvent = eventArray[i + 1]; // could be undefined (for last event)

    //console.log ('event: ' + event.beat + ' pos: ' + event.pos);


    if (event.note == 0 && !isSlur(event.type)) { // standalone rest (not in a slur)
      var keyframe0 = {};
      keyframe0.key = event.beat;
      keyframe0.pos = lastKeyStop;
      keyframes.push(keyframe0); // 'stay'

      if (!isLiftRequired(event)) { //prevEvent.type !== nextEvent.type) {  ... no need to set

        var keyframe1 = {};
        keyframe1.key = event.beat + event.duration;
        keyframe1.pos = lastKeyStop; // end of 'stay'
        keyframes.push(keyframe1);

      } else { // need to set

        var keyframe1 = {};
        var pos1 = event.beat + event.duration - 1;
        var pos2 = 0.5 * event.beat + 0.5 * nextEvent.beat;
        keyframe1.key = Math.max(pos1, pos2); // !!
        keyframe1.pos = lastKeyStop // end of 'stay'; start of 'set
        keyframes.push(keyframe1);

        var keyframe2 = {};
        keyframe2.key = nextEvent.beat; // end of 'set'
        keyframe2.pos = isDownBow(nextEvent.type) ? (nextEvent.pos - nextEvent.length / 2) : (nextEvent.pos + nextEvent.length / 2);
        keyframes.push(keyframe2);

        keyframe1.pos = keyframe2.pos;

        lastKeyStop = keyframe2.pos;
      }

    } else { // event is a note

      ////////////////
      // slur handling

      // slur starts
      if (nextEvent && isSlur(nextEvent.type) && !isSlur(event.type)) {

        eventsInSlur.push(event); // always starts with a note, NOT a rest
        slurDur = event.duration;
        slurLength = event.length;

        slurType = event.type;
        slurStart = event.beat;

        // postpone output first event

        continue; // end of this event

      }

      // slur in progress
      if (eventsInSlur.length != 0) { // signifies slur in progress

        // slur ends if ...

        if (nextEvent === undefined || // nothing to follow
          !isSlur(nextEvent.type)) { // start a new stroke

          // add the last event of this stroke
          eventsInSlur.push(event);
          if (event.note !== 0) {
            slurDur += event.duration;
            slurLength += event.length;
          }

          strokeLength = _Length(lastKeyStop, slurType, slurLength);

          // output keyframes
          var cumuRest = 0;

          for (var ii = 0; ii < eventsInSlur.length; ii++) {
            var sEvent = eventsInSlur[ii];

            if (sEvent.note != 0) { // note

              // regular note (S) or slurred staccato (SS)
              var keyframe = {};
              keyframe.key = sEvent.beat;
              if (ii == 0) { // first key.pos is most straightforward
                keyframe.pos = lastKeyStop;
              } else {
                moveRatio = (sEvent.beat - slurStart - cumuRest) / slurDur;
                keyframe.pos = lastKeyStop + _Move(slurType, moveRatio, strokeLength);
              }
              keyframes.push(keyframe);

              if (isStaccato(sEvent.type)) { // slurred staccato OR portato
                // one additional key for slurred staccato
                var keyframe1 = {};
                var dutyCycle = isPortato(sEvent.type) ? 0.8 : 0.5;
                keyframe1.key = sEvent.beat + dutyCycle * sEvent.duration;
                moveRatio = (sEvent.beat - slurStart + sEvent.duration - cumuRest) / slurDur;
                keyframe1.pos = lastKeyStop + _Move(slurType, moveRatio, strokeLength);
                keyframes.push(keyframe1);
              }

            } else { // in-slur-rest
              var keyframe = {};
              keyframe.key = sEvent.beat;

              pEvent = eventsInSlur[ii - 1];

              moveRatio = (pEvent.beat - slurStart - cumuRest + pEvent.duration) / slurDur;
              keyframe.pos = lastKeyStop + _Move(slurType, moveRatio, strokeLength);
              keyframes.push(keyframe);

              cumuRest += sEvent.duration;
            }

          }


          // last keyframe in this stroke
          lastSEvent = eventsInSlur[eventsInSlur.length - 1];

          // might not be necessary (need lastKeyStop)
          if (lastSEvent.note != 0) { // not a rest
            var keyframe = {};
            keyframe.key = lastSEvent.beat + lastSEvent.duration;
            keyframe.pos = lastKeyStop + _Move(slurType, 1.0, strokeLength);
            keyframes.push(keyframe);
          }


          // update lastKeyStop
          lastKeyStop = keyframes[keyframes.length - 1].pos;

          // terminating slur
          eventsInSlur = [];


        } else { // still in slur

          eventsInSlur.push(event);
          if (event.note != 0) { // not a in-slur-rest
            slurDur += event.duration; // rest does not count as duration
            slurLength += event.length;
          }

        }

        continue; // end of this event

      }


      // if not slurred

      var keyframe0 = {};
      // generate two keys per event (begin & end)
      keyframe0.key = event.beat;
      keyframe0.pos = lastKeyStop;
      keyframes.push(keyframe0);

      // if staccato
      if (isStaccato(event.type)) { // .type === 'DS' || event.type === 'US') {
        // if staccato (implied rest)
        var keyframe1 = {};
        keyframe1.key = event.beat + event.duration / 2;
        keyframe1.pos = isDownBow(event.type) ? event.pos + event.length / 2 : event.pos - event.length / 2;
        keyframes.push(keyframe1);
        var keyframe2 = {};
        keyframe2.key = event.beat + event.duration;
        keyframe2.pos = keyframe1.pos;
        keyframes.push(keyframe2);
        lastKeyStop = keyframe2.pos;

      } else if (isLiftRequired(event)) { // XL (lift)
        // if lift (implied rest)

        var keyframe1 = {};
        keyframe1.key = event.beat + 0.5 * event.duration;
        keyframe1.pos = isDownBow(event.type) ? event.pos + event.length / 2 : event.pos - event.length / 2;
        keyframes.push(keyframe1);

        var keyframe2 = {};
        keyframe2.key = event.beat + 0.75 * event.duration;
        keyframe2.pos = keyframe1.pos;
        keyframes.push(keyframe2);

        var keyframe3 = {};
        keyframe3.key = nextEvent.beat; // end of 'set'
        keyframe3.pos = isDownBow(nextEvent.type) ? (nextEvent.pos - nextEvent.length / 2) : (nextEvent.pos + nextEvent.length / 2);
        keyframes.push(keyframe3);

        keyframe2.pos = keyframe3.pos;

        lastKeyStop = keyframe3.pos;

      } else { // plain detache

        var keyframe1 = {};
        keyframe1.key = event.beat + event.duration;
        keyframe1.pos = isDownBow(event.type) ? event.pos + event.length / 2 : event.pos - event.length / 2;
        keyframes.push(keyframe1);
        lastKeyStop = keyframe1.pos;

      }

    }
  }
  printKeyframe(keyframes);
  return keyframes;

}

function _Move(slurType, moveRatio, strokeLength) {
  var sign = (slurType === 'D') ? 1 : -1;
  return sign * moveRatio * strokeLength;
}

function _Length(lastKeyStop, slurType, slurLength) {
  // slur starts from lastKeyStop
  var maxStroke = (slurType === 'D') ? 1 - lastKeyStop : lastKeyStop;
  return Math.min(slurLength, maxStroke);
}


function fk(theta, joints) {
  joints[0] = new THREE.Vector3(0, 0, 0);

  var m = new THREE.Matrix4();
  m.makeRotationY(theta[0]);
  m.multiply(new THREE.Matrix4().makeTranslation(UPPERARM_LEN, 0, 0));
  var localzero = new THREE.Vector3(0, 0, 0);
  localzero.applyMatrix4(m);
  joints[1].copy(localzero);

  m.multiply(new THREE.Matrix4().makeRotationY(theta[1]));
  m.multiply(new THREE.Matrix4().makeTranslation(FOREARM_LEN, 0, 0));
  localzero.set(0, 0, 0);
  localzero.applyMatrix4(m);
  joints[2].copy(localzero);

  end.copy(joints[2]);
}
