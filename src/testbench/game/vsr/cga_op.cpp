#include "game/vsr/cga_op.h"

namespace vsr {
namespace cga {

/*-----------------------------------------------------------------------------
 *  OP
 *-----------------------------------------------------------------------------*/
Rot Op::AA(const Vec &s) {
  Rot r = nga::Gen::ratio(Vec::z, s.unit());
  return nga::Gen::aa(r);
}

Rot Op::AA(const Biv &s) {
  Rot r = nga::Gen::ratio(Vec::z, s.duale().unit());
  return nga::Gen::aa(r);
}

Rot Op::AA(const Dlp &s) {
  Rot r = nga::Gen::ratio(Vec::z, Vec(s).unit());
  return nga::Gen::aa(r);
}

Rot Op::AA(const Cir &s) {
  Biv b = Round::dir(s).copy<Biv>();
  Rot r = nga::Gen::ratio(Vec::z, Op::dle(b).unit());
  return nga::Gen::aa(r);
}

Vec Op::Pos(const Dlp &s) { return Flat::loc(s, PAO, true); }

Pnt Op::Pos(const Cir &s) { return Round::loc(s); }

/*-----------------------------------------------------------------------------
 *  GEN
 *-----------------------------------------------------------------------------*/

Rot Gen::rot(const Biv &b) { return nga::Gen::rot(b); }
Rot Gen::rotor(const Biv &b) { return nga::Gen::rot(b); }

Bst Gen::bst(const Par &p) { return nga::Gen::bst(p); }
Bst Gen::boost(const Par &p) { return nga::Gen::bst(p); }

Tsd Gen::dil(const Pnt &p, VSR_PRECISION t) { return nga::Gen::dil(p, t); }
Tsd Gen::dilator(const Pnt &p, VSR_PRECISION t) { return nga::Gen::dil(p, t); }

Rot Gen::ratio(const Vec &v, const Vec &v2) { return nga::Gen::ratio(v, v2); }

Biv Gen::log(const Rot &r) { return nga::Gen::log(r); }

/*! Generate a Rot (i.e quaternion) from spherical coordinates
     @param[in] theta in xz plane from (1,0,0) in range [0,PI]
     @param[in] phi in rotated xy plane in range []
 */
Rot Gen::rot(double theta, double phi) {
  Rot rt = Gen::rot(Biv::xz * theta / 2.0);
  Rot rp = Gen::rot(Biv::xy.sp(rt) * phi / 2.0);
  return rp * rt;
}

/*! Generate a rotor from euler angles */
Rot Gen::rot(double y, double p, double r) {
  Rot yaw = Gen::rot(Biv::xz * y / 2.0);
  Rot pitch = Gen::rot(Biv::yz.spin(yaw) * p / 2.0);
  Rot tmp = pitch * yaw;
  Rot roll = Gen::rot(Biv::xy.spin(tmp) * r / 2.0);

  return roll * tmp;
}

/*! Generate a Mot from a Dual Lin Axis
    @param Dual Lin Generator (the axis of rotation, including pitch and
   period)
*/
Mot Gen::mot(const Dll &dll) {
  Dll b = dll;
  Biv B(b[0], b[1], b[2]); // Biv B(dll);

  Mot::value_t w = B.wt();

  VSR_PRECISION c = (sqrt(fabs(w)));
  VSR_PRECISION sc = sin(c);
  VSR_PRECISION cc = cos(c);

  if (ERROR(w, .00000001))
    return Mot(1, 0, 0, 0, b[3], b[4], b[5], 0); // translation only!

  B = B.unit();
  Vec t(b[3], b[4], b[5]);

  Vec tv;
  tv = Op::pj(t, B);
  Vec tw;
  tw = Op::rj(t, B);

  tv *= Math::sinc(c);

  Vec tt = tw * cc + tv;

  auto ts = B * tw; // Vec_Biv

  return Mot(cc, B[0] * sc, B[1] * sc, B[2] * sc, tt[0], tt[1], tt[2],
             ts[3] * sc);
}

Mot Gen::motor(const Dll &dll) { return mot(dll); }

// Dual Lin Generator from a Mot
// An implementation of J.Lasenby et al "Applications of Conformal Geometric
// Algebra in Computer Vision and Graphics"

Dll Gen::log(const Mot &m) {
  Drv cperp, cpara;
  Dll rq, q, tq;
  Biv b;

  q = m; // extract grade 2 part

  VSR_PRECISION ac = acos(m[0]); // angle of rotor
  VSR_PRECISION den = Math::sinc(ac);
  VSR_PRECISION den2 = ac * ac * den;

  b = ((Ori(1) <= (q * Inf(1))) / den *
       -1.0);   // bivector part - negative necessary . dll? . .
  tq = (b * q); // Make motor and extract Grade 2 part

  if (FERROR(den2)) { // Pure translation
                      //  printf("%f, %f, %f\n", ac, den, den2 );
                      //  printf("den2 = 0 in motor log\n");
    // cperp = b * -1.0;
    cperp = q; // b * tq * -1.0;// * -1.0; or q //note this used to be cpara...
               // (but was inaccurate)
  } else {
    cperp =
        (b * Drt(m[7])) / ((den2) * -1.0); // perpendicular (along line of axis)
    cpara = (b * tq) / ((den2) * -1.0); // parallel      (in plane of rotation)
  }

  Drv c = cperp + cpara;

  rq += b;
  rq += c;

  return rq;
}

Dll Gen::logMotor(const Mot &m) { return Gen::log(m); }

/*! Dual Lin Generator of Mot That Twists Dual Lin a to Dual Lin b;

*/
Dll Gen::log(const Dll &a, const Dll &b, VSR_PRECISION t) {
  Mot m = b / a;
  VSR_PRECISION n = m.rnorm();
  if (n != 0)
    m /= n;
  return Gen::log(m) * (t / 2.0);
}

/*! Generate Mot That Twists Dual Lin a to Dual Lin b;

*/
Mot Gen::ratio(const Dll &a, const Dll &b, VSR_PRECISION t) {
  // Mot m = b/a; VSR_PRECISION n = m.rnorm(); if (n!=0) m /= n; else cout <<
  // "zero mot" << endl;
  return Gen::mot(log(a, b, t)); // Gen::log( m ) * (t/2.0) );
}

/*! Generate Mot That Twists Mot a to motor b by amt t;
*/
Mot Gen::ratio(const Mot &a, const Mot &b, VSR_PRECISION t) {
  return Gen::mot(Gen::log(b / a) * t);
}

/*-----------------------------------------------------------------------------
 *  BOOSTS
 *-----------------------------------------------------------------------------*/

/*! Generate Simple Boost rotor from ratio of two dual spheres
*/
Bst Gen::ratio(const Dls &a, const Dls &b, bool bFlip) {
  Bst tbst = (b / a).runit();
  // if (tbst[0]<0)
  if (bFlip)
    tbst = -tbst; // restrict to positive <R>
  auto ss = 2 * (1 + tbst[0]);
  auto n = (ss >= 0 ? sqrt(ss) : -sqrt(-ss));
  return FERROR(n) ? Bst() : (tbst + 1) / n;
}

/*! atanh2 function for logarithm of general rotors*/
Par Gen::atanh2(const Par &p, VSR_PRECISION cs, bool bCW) {
  VSR_PRECISION norm = 1;

  auto tp = p.wt();
  auto sq = sqrt(fabs(tp));
  if (tp > 0)
    norm = asinh(sq) / sq;
  else if (tp < 0) {
    if (bCW)
      norm = -(PI - atan2(sq, cs)) / sq; // alt direction
    else
      norm = atan2(sq, cs) / sq;
  }
  return p * norm;
}

/*! Log of a simple rotor (uses atanh2, passes in boolean for direction of
 * interpolation) */
Par Gen::log(const Bst &b, bool bCW) { return atanh2(Par(b), b[0], bCW); }

/*!  Generate Conformal Transformation from circle a to circle b
     uses square root method of Dorst et Valkenburg, 2011
*/
// Con Gen::ratio(const Cir &a, const Cir &b, bool bFlip, float theta) {
//   Con trot = (b / a).runit();
//   // planar?
//   //  float planarity = (Round::carrier(a).dual().unit() ^
//   //  Round::carrier(b).dual().unit()).wt();
//   if (bFlip && trot[0] < 0) { // fabs(planarity)<=.000009 )  {
//     trot = -trot;             // restrict to positive <R> only if coplanar
//   }

//   auto rotone = trot + 1;

//   VSR_PRECISION sca = 1 + trot[0];
//   VSR_PRECISION sca2 = sca * sca;

//   Sph sph(trot);
//   auto sph2 = sph.wt();

//   if (FERROR(sca2 - sph2)) {
//     // printf("infinity of roots . . .  \n");
//     auto rotneg = (-trot) + 1;

//     Vec vec;

//     auto sizeB = nga::Round::size(b, false);
//     if (sizeB < 1000 && !FERROR(sizeB))
//       vec = Vec(Round::location(b) - Round::location(a)).unit();
//     else
//       vec = Round::vec(a, theta).unit();

//     auto dls = sph.dual();

//     auto biv = (Par(vec.copy<Tnv>()).trs(Round::location(a)) ^ dls)
//                    .undual(); //.trs(1,0,0);

//     biv = biv.runit();

//     auto test = (biv * sph - sph * biv).wt();

//     if (!FERROR((biv <= biv)[0] + 1) || (!FERROR(test))) {
//       printf("HEY NOW NOT COMMUTING\n");
//     }

//     auto ret = rotone / 2.0 + (biv * (rotneg / 2.0));
//     return ret;
//   }

//   auto sca3 = sca2 - sph2;
//   auto sqsca3 = sqrt(sca3);

//   //   cout << sca2 << " " << sph2 << " " << sca << " " << sqsca3 << endl;
//   //   sca = fabs(sca);  //<--* added this fabs in
//   auto v1 = (-sph + sca) / (2 * sca3);
//   auto v2 = (sph + (sca + sqsca3)) / sqrt(sca + sqsca3);

//   return rotone * v1 * v2;
// }

/*!  Generate Conformal Transformation from Par a to Par b
     uses square root method of Dorst et Valkenburg, 2011
*/
Con Gen::ratio(const Par &a, const Par &b, bool bFlip, float theta) {
  return ratio(a.dual(), b.dual(), bFlip, theta);
}

/*! Bivector Split
         Takes a general bivector and splits  it into commuting pairs
         will give sinh(B+-)
         inverse method for ipar below was given by dorst in personal
   correspondance
*/
vector<Par> Gen::split(const Par &par) {
  typedef decltype(Sph() + 1) SqDeriv;

  vector<Par> res;

  SqDeriv h2 = par * par;
  auto hh2 = Sph(h2).wt();
  auto ipar = (-Sph(h2) + h2[0]) / (h2[0] * h2[0] - hh2);

  // scalar ||f||^2
  auto tmp2 = ((h2 * h2) - (h2 * 2 * h2[0]))[0];
  auto ff4 = FERROR(tmp2) ? 0 : pow(-tmp2, 1.0 / 4);
  auto wt = ff4 * ff4;
  //  cout << par << endl;
  //  cout << h2 << endl;
  //  cout << "SPLIT WEIGHT: " << -tmp2 << " " << h2[0] << " " << wt << endl;
  if (FERROR(wt)) {
    if (FERROR(h2[0])) {
      // cout << h2 << endl;
      //  printf("no real splitting going on\n"); //<-- i.e. interpolation of
      //  null point pairs
      res.push_back(par);
      // res.push_back(Par());
      return res;
    } else {
      //  printf("(adding random value and retrying)");
      static Par dp(.0001, .0006, .0004, .0002, .0008, .0006, .0003, .0007,
                    .0001, .0001);
      return split(par + dp);
    }
  }

  auto iha = ipar * wt;
  auto fplus = iha + 1;
  auto fminus = -iha + 1;

  Par pa = par * fplus * .5;
  Par pb = par * fminus * .5;

  res.push_back(pa);
  res.push_back(pb);

  return res;
}

vector<Par> Gen::split(const Con &rot) {
  // 1. Get Exterior Derivative
  Sph quad(rot); // grade 4 part

  auto tmp = quad + (-rot[0]); // scalar + quadvector
  // half the exterior deriv is sinh(B+/-)
  auto deriv = Par(tmp * Par(rot)) * 2; // quad parts are zero here.
  // find commuting split of that
  return split(deriv);
}

/*! Split Log of General Conformal Rot */
vector<Par> Gen::log(const Con &rot) {
  vector<Par> res;

  // 0. Some Terms for later on
  // R^2
  auto sqrot = rot * rot;
  //<R>2
  Par rot2(sqrot);

  // 1. Get Exterior Derivative
  Sph quad(rot); // grade 4 part

  auto tmp = quad + (-rot[0]); // scalar + quadvector
  // half the exterior deriv is sinh(B+/-)
  auto deriv = Par(tmp * Par(rot)) * 2; // quad parts are zero here.
  // find commuting split of that
  auto v = split(deriv);

  // get cosh (see p96 of ref)
  auto sp = v[0].wt(); //(v[0]<=v[0])[0];
  auto sm = v[1].wt(); //(v[1]<=v[1])[0];

  VSR_PRECISION coshp = FERROR(sm) ? sqrot[0] : -(rot2 <= !v[1])[0];
  VSR_PRECISION coshm = FERROR(sp) ? sqrot[0] : -(rot2 <= !v[0])[0];

  // 5.27 on p96 of Dorst ref
  res.push_back(atanh2(v[0], coshp, false) * -.5);
  res.push_back(atanh2(v[1], coshm, false) * -.5);

  return res;
}

/*! Split Log from a ratio of two Cirs */
vector<Par> Gen::log(const Cir &ca, const Cir &cb, bool bFlip,
                     VSR_PRECISION theta) {
  return log(ratio(ca, cb, bFlip, theta));
}
/*! Split Log from a ratio of two Cirs */
vector<Par> Gen::log(const Par &ca, const Par &cb, bool bFlip,
                     VSR_PRECISION theta) {
  return log(ratio(ca, cb, bFlip, theta));
}

/*! General Conformal Transformation from a split log*/
Con Gen::con(const vector<Par> &log, VSR_PRECISION amt) {
  Con con(1);
  for (auto &i : log) {
    con *= Gen::bst(i * -amt);
  }
  return con;
}

/*! General Conformal Transformation from a split log*/
Con Gen::con(const vector<Par> &log, VSR_PRECISION amtA, VSR_PRECISION amtB) {
  Con tmp = Gen::bst(log[0] * -amtA);
  if (log.size() > 1)
    tmp *= Gen::bst(log[1] * -amtB);
  return tmp;
}

/*! General Conformal Transformation from two circles */
Con Gen::con(const Cir &ca, const Cir &cb, VSR_PRECISION amt) {
  return con(log(ca, cb), amt);
}

/* General Conformal Transformation from two circles and two weights */
Con Gen::con(const Cir &ca, const Cir &cb, VSR_PRECISION amtA,
             VSR_PRECISION amtB) {
  return con(log(ca, cb), amtA, amtB);
}

/*-----------------------------------------------------------------------------
 *  ROTORS
 *-----------------------------------------------------------------------------*/
/*!
* \brief  generate a rotor transformation from a euclidean bivector
*/
Rot Gen::xf(const Biv &b) { return Gen::rot(b); }
/*!
 *  \brief  generate a motor transformation from a dual line
 */
Mot Gen::xf(const Dll &dll) { return Gen::mot(dll); }
/*!
 *  \brief  generate a dilation transformation from a flat point
 */
Dil Gen::xf(const Flp &flp) { return Gen::dil(Pnt(flp), flp[3]); }

/*!
 *  \brief  generate a boost transformation from a point pair
 */
Bst Gen::xf(const Par &p) { return Gen::bst(p); }

/*-----------------------------------------------------------------------------
 *  PAIRS
 *-----------------------------------------------------------------------------*/
/// Par on Sph in v direction
Par Construct::pair(const Dls &s, const Vec &v) { return Round::produce(s, v); }

/*!
 *  \brief Pnt Par at x,y,z with direction vec (default Y) and radius r
 * (default 1)
 */
Par Construct::pair(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z, Vec vec,
                    VSR_PRECISION r) {
  return Round::produce(Round::dls(r * -1, x, y, z), vec); // a ^ b ^ c;
}

/*-----------------------------------------------------------------------------
 *  POINTS
 *-----------------------------------------------------------------------------*/
/*!
 *  \brief  First point of point pair pp
 */
Pnt Construct::pointA(const Par &pp) {
  return Round::location(Round::split(pp, true));
}

/*!
 *  \brief  Second point of point pair pp
 */
Pnt Construct::pointB(const Par &pp) {
  return Round::location(Round::split(pp, false));
}

/// Pnt on Cir at theta t
Pnt Construct::point(const Cir &c, VSR_PRECISION t) {
  return Round::point(c, t);
}
/// Pnt on Sph in v direction
Pnt Construct::point(const Dls &s, const Vec &v) {
  return pointA(pair(s, v)).null();
}

/// Pnt from x,y,z
Pnt Construct::point(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z) {
  return Round::null(x, y, z);
}
/// Pnt from vec
Pnt Construct::point(const Vec &v) { return v.null(); }

/// Pnt on line l closest to p
Pnt Construct::point(const Lin &line, const Pnt &p) {
  return Round::null(Flat::location(line, p, false));
}
/// Pnt on dualline l closest to p
Pnt Construct::point(const Dll &dll, const Pnt &p) {
  return Round::null(Flat::location(dll, p, true));
}

/*-----------------------------------------------------------------------------
 *  CIRCLES
 *-----------------------------------------------------------------------------*/
/*!
 *  \brief  Cir at origin in plane of bivector B
 */
Cir Construct::circle(const Biv &B) {
  return Round::produce(Round::dls(1, 0, 0, 0), B); // a ^ b ^ c;
}

/*!
 *  \brief  Cir at point p with radius r, facing direction biv
*/
Cir Construct::circle(const Pnt &p, VSR_PRECISION r, const Biv &biv) {
  return Round::produce(Round::dls(p, r * -1.0), biv);
}

/// Cir at origin with normal v and radius r (default r=1.0)
Cir Construct::circle(const Vec &v, VSR_PRECISION r) {
  return Round::produce(Round::dls(r * -1, 0, 0, 0), Op::dle(v)); // a ^ b ^ c;
}
/// Cir at x,y,z facing in biv direction
Cir Construct::circle(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z,
                      Biv biv, VSR_PRECISION r) {
  return Round::produce(Round::dls(r * -1, x, y, z), biv); // a ^ b ^ c;
}
/*-----------------------------------------------------------------------------
 *  HYPERBOLIC AND SPHERICAL LINES
 *-----------------------------------------------------------------------------*/
/// Hyperbolic line through two points
Cir Construct::hline(const Pnt &a, const Pnt &b) { return a ^ b ^ EP; }
/// Spherical line through two points
Cir Construct::sline(const Pnt &a, const Pnt &b) { return a ^ b ^ EM; }

/*-----------------------------------------------------------------------------
 *  SPHERES
 *-----------------------------------------------------------------------------*/
/// Sph at x,y,z with radius r (default r=1.0)
Dls Construct::sphere(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z,
                      VSR_PRECISION r) {
  return Round::dls(r, x, y, z);
}
/// Sph at point p with radius r (default r=1.0)
Dls Construct::sphere(const Pnt &p, VSR_PRECISION r) {
  return Round::dls(p, r);
}

/*-----------------------------------------------------------------------------
 *  PLANES
 *-----------------------------------------------------------------------------*/

/// Dual plane with normal and distance from center
Dlp Construct::plane(VSR_PRECISION a, VSR_PRECISION b, VSR_PRECISION c,
                     VSR_PRECISION d) {
  return Dlp(a, b, c, d);
}
/// Dual plane from vec and distance from center
Dlp Construct::plane(const Vec &v, VSR_PRECISION d) { return v + Inf(d); }
/// Direct plane through three points
Pln Construct::plane(const Pnt &a, const Pnt &b, const Pnt &c) {
  return a ^ b ^ c ^ Inf(1);
}

/*-----------------------------------------------------------------------------
 *  LINES
 *-----------------------------------------------------------------------------*/
/*!
 *  \brief  Dll axis of circle c
 */
Dll Construct::axis(const Cir &c) { return (Inf(-1) <= c).runit(); }

/// Lin from two Vecs
Lin Construct::line(const Vec &a, const Vec &b) {
  return point(a[0], a[1], a[2]) ^ Vec(b[0], b[1], b[2]) ^ Inf(1);
}

/// Direct line through origin
Lin Construct::line(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z) {
  return Ori(1) ^ Vec(x, y, z) ^ Inf(1);
}
/// Direct line through origin
Lin Construct::dualLin(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z) {
  return line(x, y, z).dual();
}

/// Direct line through two points
Lin Construct::line(const Pnt &a, const Pnt &b) { return a ^ b ^ Inf(1); }
/// Direct line through point a in direction b
Lin Construct::line(const Pnt &a, const Vec &b) { return a ^ b ^ Inf(1); }

/// Squared Distance between a line and a point
VSR_PRECISION Construct::distance(const Lin &lin, const Pnt &pnt) {
  return (pnt <= lin.dual())[0] * -2.0;
}

/* Lin line(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION z){ */
/*   return point(a[0], a[1], a[2]) ^ Vec(b[0], b[1], b[2]) ^ Inf(1); */
/* } */

#pragma mark COINCIDENCE_FUNCTIONS

/// circle intersection of dual spheres
Cir Construct::meet(const Dls &s, const Dls &d) { return (s ^ d).dual(); }

/// circle intersection of dual sphere and direct plane
Cir Construct::meet(const Dls &s, const Dlp &d) { return (s ^ d).dual(); }
/// circle intersection of dual spehre and direct plane
Cir Construct::meet(const Dls &s, const Pln &d) {
  return (s ^ d.dual()).dual();
}
// circle intersection of direct sphere and dual plane
Cir Construct::meet(const Sph &s, const Dlp &d) {
  return (s.dual() ^ d).dual();
}
// circle intersection of direct sphere and direct plane
Cir Construct::meet(const Sph &s, const Pln &d) {
  return (s.dual() ^ d.dual()).dual();
}

// normalized and nulled point intersection of line and dual plane
Pnt Construct::meet(const Lin &lin, const Dlp &dlp) {
  Flp flp = ((lin).dual() ^ dlp).dual();
  return (flp / flp[3]).null();
}
// normalized and nulled point intersection of dualline and dual plane
Pnt Construct::meet(const Dll &dll, const Dlp &dlp) {
  auto flp = (dll ^ dlp).dual();
  return flp.null();
}

// Pnt intersection of two lines
Pnt Construct::meet(const Lin &la, const Lin &lb) {
  Lin r = la.reflect(lb);
  Lin r2 = (la - r.unit()).unit();
  Pnt pori = Flat::loc(r2, Ori(1), false);
  Pnt tp = pori.re(lb);
  return (((tp / tp[3]) + pori) / 2.0).null();
}
/* //test for origin */
/* auto t = Ori(1) ^ la; Sca(t.wt()).vprint(); */
/* (t.dual() ^ lb.dual() ).dual().vprint(); */
/* if (t.wt() != 0 ) { */
/*  return ( t.dual() ^ lb.dual() ).dual(); */
/* } else { */
/*  auto t2 = (Ori(1) ^ lb ); */
/*  if ( t2.wt() != 0 ) return ( la.dual() ^ t2.dual() ).dual(); */
/*  else return Flp(); */
/* } */
//}

// point pair intersection of circle and Dual plane
Par Construct::meet(const Cir &cir, const Dlp &dlp) {
  return ((cir).dual() ^ dlp).dual();
}

// point pair intersection of circle and Dual sphere
Par Construct::meet(const Cir &cir, const Dls &s) {
  return ((cir).dual() ^ s).dual();
}

#pragma mark HIT_TESTS

/*!
 *  \brief  hit tests between point and pair (treats pair as an "edge")
 */
bool Construct::hit(const Pnt &pnt, const Par &par) {
  // if inside surround < 0
  if ((pnt <= Round::sur(par))[0] < 0) {
    if ((pnt ^ par ^ Inf(1)).wt() == 0.0) {
      return true;
    }
  }
  return false;
}

/*!
 *  \brief  hit tests between point and circle (treats circle as "disc")
 */
bool Construct::hit(const Pnt &p, const Cir &cir) {
  if ((p <= Round::sur(cir))[0] > 0) {
    if (fabs((p ^ Round::car(cir)).wt()) < .00001) {
      return true;
    }
  }
  return false;
}

double Construct::squaredDistance(const Pnt &a, const Pnt &b) {
  return Round::sqd(a, b);
}

#pragma mark HYPERBOLIC_FUNCTIONS
/*-----------------------------------------------------------------------------
 *  hyperbolic functions (see alan cortzen's document on this)
 *-----------------------------------------------------------------------------*/

/*!
 *  \brief  hyperbolic normalization of a conformal point
 */
Pnt Construct::hnorm(const Pnt &p) { return -(p / (EP <= p)); }

/*!
 *  \brief  hyperbolic distance between two conformal points
 */
double Construct::hdist(const Pnt &pa, const Pnt &pb) {
  return acosh(1 - (hnorm(pa) <= hnorm(pb))[0]);
}

/*!
 *  \brief  hyperbolic translation transformation generator between two
 * conformal points
 */
Par Construct::hgen(const Pnt &pa, const Pnt &pb, double amt) {
  double dist = hdist(pa, pb);             //<-- h distance
  auto hline = pa ^ pb ^ EP;               //<-- h line (circle)
  auto par_versor = (EP <= hline).runit(); //<-- h trans generator (pair)
  // par_versor /= par_versor.rnorm();   //<-- normalized ...
  return par_versor * dist * amt * .5; //<-- and ready to be applied
}

/*!
 *  \brief  hyperbolic spin transformation from pa to pb by amt (0,1)
 */
Pnt Construct::hspin(const Pnt &pa, const Pnt &pb, double amt) {
  return Round::loc(pa.boost(hgen(pa, pb, amt)));
  // versor * dist * amt * .5) );
}

} // cga::
} // vsr::
