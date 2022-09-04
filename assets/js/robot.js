/*
    Matrix Robot
    Created By PJoyet
*/

class Robot {
  constructor(position, liaison, segments, DH_d = false, DH_alpha = false) {
    this.position = position;
    this.liaison = liaison;
    this.segments = segments;

    if (DH_d == false) {
      this.DH_d = [0];
      for (let index = 0; index < liaison.length - 1; index++) {
        this.DH_d.push(0);
      }
    }
    if (DH_alpha == false) {
      this.DH_alpha = [0];
      for (let index = 0; index < liaison.length - 1; index++) {
        this.DH_alpha.push(0);
      }
    }
    this.DH = { "theta": this.liaison, "d": this.DH_d, "a": this.segments, "alpha": this.DH_alpha };

  }

  Calc_DH(theta,d,a,alpha){
    let TH = math.matrix([
      [Math.cos(theta),-Math.sin(theta)*Math.cos(alpha), Math.sin(theta)*Math.sin(alpha),a*Math.cos(theta)],
      [Math.sin(theta), Math.cos(theta)*Math.cos(alpha),-Math.cos(theta)*Math.sin(alpha),a*Math.sin(theta)],
      [              0,                 Math.sin(alpha),                 Math.cos(alpha),                d],
      [              0,                               0,                               0,                1]]);
    return TH;
  }
  Calc_TH(start,stop) {
    let TH = math.identity(4);  
    TH = math.subset(TH,math.index([0,1,2],3),this.position);
    for (let index = start; index < stop; index++) {
      let DH = this.Calc_DH(this.DH.theta[index],this.DH.d[index],this.DH.a[index],this.DH.alpha[index]);
      TH = math.multiply(TH,DH);
    }
  return TH;
}
  getPosA(){
    let cal = this.Calc_TH(0,this.liaison.length);
    return math.subset(cal, math.index(math.range(0,3), 3));
  }

  getAllPosA(self){
  let result = math.zeros(3,this.liaison.length);
  for (let index = 0; index < this.liaison.length; index++) {
    let Coord =math.subset(this.Calc_TH(0,index), math.index(math.range(0,3), 3));
    result = math.subset(result,math.index(math.range(0,3),index),Coord)
  }
  return result;
  }

  Jacobienne(){
    let J = math.zeros(6, this.liaison.length) ;
    let vec = [0,0,1];
    let endEffect = this.getPosA();
    let LiaisCoord, V, omega, Jvec; 
    let LiaisRot = math.identity(3);

    for (let index = 0; index < this.liaison.length; index++) {
      LiaisCoord = math.subset(this.Calc_TH(0,index), math.index(math.range(0,3), 3));
      LiaisRot = math.subset(this.Calc_TH(0,index), math.index(math.range(0,3), math.range(0,3)));
      V = math.subset(LiaisRot, math.index(math.range(0,3), 2));
      omega = math.cross(V,math.subtract(endEffect, LiaisCoord));
      Jvec = math.concat(math.transpose(omega),V,0);
      J = math.subset(J,math.index(math.range(0,6),index),Jvec);
    }
    return J
  }

  simule(pos,gain=1,dt=0.1){
    pos = [pos];
    let E = math.subtract(pos,math.transpose(this.getPosA()));
    E = math.multiply(E,gain);
    let J = math.subset(this.Jacobienne(),math.index(math.range(0,3),math.range(0,this.liaison.length)));
    let piJ = math.pinv(J);
    let dtheta = math.multiply(piJ,math.transpose(E));
    dtheta = math.multiply(math.transpose(dtheta),dt)

    this.DH["theta"]=[this.DH["theta"][0]+dtheta.subset(math.index(0,0)),
                      this.DH["theta"][1]+dtheta.subset(math.index(0,1)),
                      this.DH["theta"][2]+dtheta.subset(math.index(0,2))];
  }

}


