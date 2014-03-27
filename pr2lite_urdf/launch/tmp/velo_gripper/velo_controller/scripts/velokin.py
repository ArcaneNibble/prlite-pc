#!/usr/bin/env pytho

import sys,os,re

import pylab as p
p.interactive(True)

def mm_array(x,y):
    return p.array([x,y])/1000.0

def twoDang(ang ):
    if isinstance(ang,p.ndarray): ang = p.arctan2(ang[1],ang[0])
    return ang

def hypleg(x,y=None):
    if y is None and isinstance(x,p.ndarray): x,y=x
    return p.sqrt(p.fabs(p.norm(x)**2-p.norm(y)**2))

def twoDmag_ang(mag,ang):
    mag = p.norm(mag)
    ang = twoDang(ang)
    return p.array([mag*p.cos(ang),mag*p.sin(ang)])

def propkin( pp ):
    pa = p.array([p.array(x) for x in pp])
    ppt = [p.sum(pa[:i+1],0) for i in range(len(pa))]
    pxy = p.array(zip(*ppt))
    #print pxy
    return pxy


R2D = 180/p.pi
D2R = p.pi/180

fz = 0
q0 = 20.0*D2R

def  q2tg( q ):
    global fz
    if not fz:
        qq = q
        q=q0

    slack = 1.0/1000
    t0  = 10.7/1000 #10.9124/1000
    P   = mm_array(5.725,13.215)
    PA  = mm_array(10.198,-2.0)
    B   = mm_array(15.087,5.585)
    C   = mm_array(51.43,3.878)
    D   = mm_array(60,0)
    E   = mm_array(-6.9,41)
    W   = mm_array(17.5,-6)

    rp = 4.0/1000    # POM is 3.96
    rb = 3.2/2/1000  # metal is 3.175/2, but loose, then tendon
    rc = rb
    rd = 3.1/1000
    qd2= -p.pi       # Abitrary.  Needs to be enough to stay wrapped.
    re0= 5.4/1000
    re1= 3.2/1000

    beta = twoDang(B)
    AB = twoDmag_ang(B,q+beta)
    PB = PA+AB
    gamma = twoDang(C)
    AC = twoDmag_ang(C,q+gamma)
    BC = AC-AB
    delta = twoDang(D)
    AD = twoDmag_ang(D,q+delta)
    CD = AD-AC
    DE = E
    WA = W
    WE = WA+AD+DE
    g = 2*WE[0]

    qp2 = twoDang(PB) + p.arccos((rp+rb)/p.norm(PB))
    fPB = hypleg(PB,rp+rb)
    qb1 = qp2 + p.pi

    fBC = p.norm(BC)
    qb2 = twoDang(BC)+3*p.pi/2
    sB  = rb*(qb2-qb1)

    qc1 = qb2
    qc2 = twoDang(CD) + 2*p.pi - p.arccos((rc+rd)/p.norm(CD))
    fCD = hypleg(CD,rc+rd)
    sC  = rc*(qc2-qc1)
    qd1 = qc2 - p.pi
    sD  = rd*(qd1-qd2)

    fBD = fPB + sB + fBC + sC + fCD + sD

    if not fz:
        # COMPUTE TOTAL TENDON LENGTH AT OPEN GRIPPER
        TP   = P+p.array([0.0,slack])
        fTP  = hypleg(TP,rp)
        qp1  = twoDang(TP) + p.pi - p.arccos(rp/p.norm(TP))
        sP   = rp*(qp1-qp2)
        fz   = fTP + sP + fBD
        # OUTPUT INFO ABOUT START POINT
        print ("t0=%.4f, TPz=[%.4f,%.4f], fTPz=%.4f, qp1=%.1f, fz=%.4f"%(t0,TP[0],TP[1],fTP,qp1*R2D,fz))

    # APPROX qp1 as constant
    qp1_approx = 173.0 * D2R
    qp1 = qp1_approx
    sP  = rp*(qp1-qp2)
    P1x = rp*p.cos(qp1)
    P1y = rp*p.sin(qp1)
    fTP = fz-sP-fBD
    fTPx = P[0] + P1x
    fTPy = hypleg(fTP,fTPx)
    t = P[1] + t0 + slack + P1y - fTPy

    # PLOT THE "SKELETON" OF IMPORTANT POINTS TO SANITY CHECK 
    skel = propkin([[0.0,t-t0],P+p.array([0.0,t0-t]),PA,AB,BC,CD,DE])
    p.figure(10); p.axis("equal")
    p.plot(skel[0,:],skel[1,:])

#    ####################
#    # FORCES FOR SPRING
#    Ft  = 1.0  # unit force
#
#    Fb  = twoDmag_ang( 2*Ft * p.sin((qb2-qb1)/2), (qb2+qb1)/2 - p.pi)
#    TQb = p.cross(AB,Fb)
#
#    Fc  = twoDmag_ang( 2*Ft * p.sin((qc2-qc1)/2), (qc2+qc1)/2 - p.pi)
#    TQc = p.cross(AC,Fc)
#
#    # NOTE: NET TORQUE ABOUT DISTAL IS ZERO.
#    # WE DO NOT SOLVE TORQUE ABOUT DISTAL PROBLEM.
#    # REACTION FORCES ON PROXIMAL LINK AXLE CAUSE TORQUES ON PROXIMAL
#
#    # FLEXOR TENDON FORCE ON DISTAL, TRANSFERED TO AXLE
#    Ff1 = twoDmag_ang( Ft, qd1 + p.pi/2 )
#    TQf = p.cross(AD,Ff1)
#
#    # SPRING TENDON FORCE ON DISTAL, TRANSFERED TO AXLE
#    Fs  = 1.0  # unit force
#    Fe1 = twoDmag_ang( Fs, q + p.arcsin((re0-re1)/p.norm(AD)) + p.pi )
#    TQFe= p.cross(AD,Fe1)
#
#    Ft_per_Fs = -TQFe/(TQb + TQc + TQf)

    ######################
    # Moment arm for proximal
    PP1 = twoDmag_ang( rp, qp1 )
    AB1 = AB + twoDmag_ang( rb, qb1 )
    PB1 = PA + AB1
    dirB1P1 = (PP1 - PB1)/p.norm(PP1 - PB1)
    mF = p.cross( AB1, dirB1P1 )

    return (t,q,g, mF)


aa = (p.arange(83.0)+20.0) * D2R

u = p.amap(q2tg,aa)

ut = u[:,0]
uq = u[:,1]; uqd = uq*R2D
ug = u[:,2]
ufma= u[:,3]

dt0 = ut[1]-ut[0]
dt1 = ut[-1]-ut[-2]
dg0 = ug[1]-ug[0]
dg1 = ug[-1]-ug[-2]

##############
# label Skeleton Plot
p.figure(10); p.grid(True)
p.title('Skeleton of Important Points')

ut_step = .0005
ut_lin = p.arange(ut[0]+ut_step,0.020,ut_step)
dgdt = dg0/dt0
ug_lin = ug[0]+(ut_lin-ut[0])*dgdt
utt = p.append(ut_lin[::-1],ut)
ugg = p.append(ug_lin[::-1],ug)

cl2g = p.polyfit(utt,ugg,4)
tl2g = p.arange(-2.0,20.1,.1)/1000.0
gl2g = p.polyval(cl2g,tl2g)

cg2l = p.polyfit(ugg,utt,4)
gg2l = p.arange(-19.0,226.1,1.0)/1000.0
tg2l = p.polyval(cg2l,gg2l)

##############
# tendon vs. Angle
p.figure(0); p.grid(True)
p.plot(uqd,1000*ut)
p.title('Tendon (mm) vs. Proximal angle (deg)')
##############
# gap vs. Angle
p.figure(1); p.grid(True)
p.plot(uqd,1000*ug)
p.title('Gap (mm) vs. Proximal angle (deg)')

##############
# tendon vs. gap and polyfits
p.figure(2); p.grid(True)
p.plot(1000*ut,1000*ug,label='kin theory')
p.plot(1000*ut_lin,1000*ug_lin,label='linear extension')
p.plot(1000*tl2g,1000*gl2g,label='l2g polynomial')
p.plot(1000*tg2l,1000*gg2l,label='g2l polynomial')
p.legend(loc=4)
p.title('Gap (mm) vs. Tendon (mm)')

##############
# Flexor Moment Arm vs. gap
cg2fma = p.polyfit(ug,ufma,4)
gg2fma = p.arange(-19.0,160.1,1.0)/1000.0
sg2fma = p.polyval(cg2fma,gg2fma)

p.figure(22); p.grid(True)
p.plot(1000*ug,ufma,label='kin theory')
p.plot(1000*gg2fma,sg2fma,label='g2fma polynomial')
p.legend(loc=3)
p.title('Flexor Moment Arm (mm) vs. Gap (mm)')


##############
# OUTPUT COEFFS IN yaml AND urdf FORMATS FOR CUT/PASTE TO WHERE NEEDED
n=len(cl2g)
print("")
print("polynomials:")
for i in range(n):  print("  l2g_%d: % .11g"%(i,cl2g[n-i-1]))
print("")
for i in range(n):  print("  g2l_%d: % .11f"%(i,cg2l[n-i-1]))
print("")
for i in range(n):  print("  g2ed_%d: % .11g"%(i,cg2fma[n-i-1]))
print("")
print("")
for i in range(n):  print('      l2g_coeffs_%d="${%0.11g}"'%(i,cl2g[n-i-1]))
print("")
for i in range(n):  print('      g2l_coeffs_%d="${%0.11f}"'%(i,cg2l[n-i-1]))
print("")
# NOTE, FOR HISTORICAL REASONS, ceoffs g2fma are called g2ed 
# (Flexor Moment Arm aka. Effective Distance)
for i in range(n):  print('      g2ed_coeffs_%d="${%0.11g}"'%(i,cg2fma[n-i-1]))
print("")


