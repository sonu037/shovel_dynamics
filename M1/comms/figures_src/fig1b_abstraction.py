"""
Figure 1(b) - Stage-1 two-degree-of-freedom abstraction.
Geometry in CONFIG; labels placed by the shapely engine in layout.py.
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from layout import Placer, Callouts, audit

NAVY="#1F3864"; OCHRE="#C8A15A"; OCHRE_D="#8A6D2F"; RED="#B03A2E"
GREY="#8A8A8A"; PALE="#E4EAF3"; STEEL="#5B739B"
DIP_F="#FFFFFF"; DIP_E="#1A1A1A"                 # dipper: white body, dark edge
F2="#0F7B6C"; F3="#7B3FA0"; F4="#C2185B"; FW="#555555"   # frames {o2} {o3} {o4} {W}
Q3=45.0; D4=6.10; C_OFF=-1.32/1.0
plt.rcParams.update({"font.family":"serif","mathtext.fontset":"cm"})
U=lambda d:(lambda t:np.array([np.cos(t),np.sin(t)]))(np.radians(d))
N=lambda v:np.array([-v[1],v[0]])

TICKS=[]
def triad(ax,O,e1,e2,L,c,n1,n2,z=14,lw=1.5):
    """Draw a 2-D frame triad; axis ticks are queued for collision-aware placement."""
    for e,other,nm in ((e1,e2,n1),(e2,e1,n2)):
        ax.annotate("",xy=O+L*e,xytext=O,
                    arrowprops=dict(arrowstyle="-|>",color=c,lw=lw,shrinkA=0,shrinkB=0),zorder=z)
        p=N(e); sgn=-1.0 if float(p@other)>0 else 1.0
        TICKS.append((O+L*1.02*e, nm, c, tuple(0.7*e+0.7*sgn*p)))
    ax.add_patch(Circle(O,0.075,fc="#FFFFFF",ec=c,lw=1.2,zorder=z+1))
    ax.add_patch(Circle(O,0.026,fc=c,zorder=z+2))          # z out of page

def curved_dipper(ax,E,h,hn,c_edge,c_face,z=7):
    """Dipper drawn with a curved back and rounded heel - real dippers are not sharp L shapes."""
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    P=lambda a,b: E+a*h+b*hn
    verts=[P(0.92,0.54), P(0.92,-0.30), P(0.72,-0.66), P(0.10,-0.72),
           P(-0.34,-0.44), P(-0.34,0.36), P(-0.10,0.58), P(0.92,0.54)]
    codes=[Path.MOVETO,Path.LINETO,Path.CURVE3,Path.CURVE3,
           Path.CURVE3,Path.LINETO,Path.CURVE3,Path.CURVE3]
    ax.add_patch(PathPatch(Path(verts,codes),fc=c_face,ec=c_edge,lw=1.8,zorder=z))

def beam(ax,p,q,w,fc,ec,lw=1.6,z=5):
    d=(q-p)/np.linalg.norm(q-p); nn=N(d)
    ax.add_patch(Polygon([p+w*nn,q+w*nn,q-w*nn,p-w*nn],closed=True,fc=fc,ec=ec,lw=lw,zorder=z))

fig=plt.figure(figsize=(15.4,9.2))
gs=fig.add_gridspec(1,2,width_ratios=[1.0,0.30],wspace=0.01)
ax=fig.add_subplot(gs[0]); axk=fig.add_subplot(gs[1])
ax.set_aspect("equal"); ax.axis("off"); axk.axis("off")
h=U(Q3); hn=N(h); O=np.array([1.30,1.10])

# ground + welded boom
ax.plot([0.15,2.45],[0.42,0.42],color=GREY,lw=1.7,zorder=2)
for x in np.linspace(0.30,2.32,9): ax.plot([x,x-0.17],[0.42,0.20],color=GREY,lw=0.9,zorder=2)
ax.add_patch(Polygon([[O[0]-0.46,0.42],[O[0]+0.46,0.42],[O[0],O[1]]],closed=True,
                     fc=PALE,ec=STEEL,lw=1.5,zorder=3))
# handle, saddle, dipper
a1=O-1.35*h; b1=O+D4*h
beam(ax,a1,b1,0.21,OCHRE,OCHRE_D,1.6,z=6)
ax.add_patch(Polygon([O+0.48*h+0.42*hn,O+0.48*h-0.42*hn,O-0.48*h-0.42*hn,O-0.48*h+0.42*hn],
                     closed=True,fc="#FFFFFF",ec=NAVY,lw=1.9,zorder=8))
ax.add_patch(Circle(O,0.12,fc=RED,zorder=9))
E=b1+0.18*h
ax.add_patch(Polygon([E+0.88*h+0.56*hn,E+0.88*h-0.68*hn,E-0.30*h-0.68*hn,E-0.30*h+0.56*hn],
                     closed=True,fc=DIP_F,ec=DIP_E,lw=2.0,zorder=7))
com=O+(D4-1.32)*h
ax.add_patch(Circle(com,0.17,fc="#FFFFFF",ec=RED,lw=1.7,zorder=10))
ax.add_patch(Circle(com,0.055,fc=RED,zorder=11))
# q3 arc
thq=np.linspace(0,np.radians(Q3),60); arcq=O+1.85*np.c_[np.cos(thq),np.sin(thq)]
ax.plot([O[0],O[0]+2.85],[O[1],O[1]],color=GREY,lw=1.0,ls=(0,(5,4)),zorder=4)
ax.plot(arcq[:,0],arcq[:,1],color=RED,lw=1.8,zorder=9)
ax.annotate("",xy=arcq[-1],xytext=arcq[-6],arrowprops=dict(arrowstyle="-|>",color=RED,lw=1.8),zorder=9)
# dimension rails
def rail(p,q,off,c,lw=1.35):
    d=(q-p)/np.linalg.norm(q-p); nn=N(d)
    ax.annotate("",xy=q+off*nn,xytext=p+off*nn,
                arrowprops=dict(arrowstyle="<|-|>",color=c,lw=lw),zorder=9)
    for pt in (p,q): ax.plot(*zip(pt,pt+off*nn),color=c,lw=0.6,ls=(0,(3,3)),zorder=8)
rail(O,b1,-0.70,NAVY); rail(O,com,-1.55,RED,1.6)
c1=O+1.45*h+0.46*hn; c2=O+3.25*h+0.46*hn
ax.annotate("",xy=c2,xytext=c1,arrowprops=dict(arrowstyle="<|-|>",color=RED,lw=1.8),zorder=10)
# ---- coordinate frames (following the source's assignment: o2 and o3 coincide) ----
W0=np.array([-2.05,-1.55])
triad(ax,W0,np.array([1.,0.]),np.array([0.,1.]),0.95,FW,r"$x_0$",r"$y_0$")

triad(ax,O,np.array([1.,0.]),np.array([0.,1.]),1.05,F2,r"$x_2$",r"$y_2$")
triad(ax,O,h,hn,1.05,F3,r"$x_3$",r"$y_3$")
triad(ax,b1,h,hn,0.85,F4,r"$x_4$",r"$y_4$")

# gravity
ax.annotate("",xy=(8.9,2.3),xytext=(8.9,3.9),arrowprops=dict(arrowstyle="-|>",color=NAVY,lw=2.2),zorder=9)


ax.text(-2.6,9.15,"(b)  Stage-1 abstraction: two degrees of freedom",color=NAVY,fontsize=16,fontweight="bold")
ax.text(-2.6,8.62,r"$q_3$ saddle rotation (revolute)  ·  $d_4$ crowd extension (prismatic)  ·  boom welded to ground"
                  "\nswing and hoist-rope actuation enter at milestones M5–M6",color=GREY,fontsize=11.2,va="top")
ax.set_xlim(-4.4,11.8); ax.set_ylim(-3.8,9.6)

fig.canvas.draw(); ren=fig.canvas.get_renderer()

# math symbols stay on the drawing (few, short); every worded name becomes a callout
P=Placer(ax,fig,ren,pad=8.0); P.freeze_obstacles()
L=P.add
for pos,nm,c,pref in TICKS: L(pos,nm,c,11.5,leader=False,prefer=[pref])
L(np.array([8.9,3.1]),r"$g$",NAVY,17,leader=False,prefer=[(1,0)])
L(arcq[len(arcq)//2],r"$q_3$",RED,16,leader=False,prefer=[(0.7,-0.3)])
L((c1+c2)/2,r"$\dot d_4$  (motion),   $F_4$  (crowd force)",RED,13.5,prefer=[(-0.5,1),(0,1)])
L((O+b1)/2-0.70*N(h),r"$d_4$",NAVY,14.5,leader=False,prefer=[(0.5,-1)])
L((O+com)/2-1.55*N(h),r"$r=d_4+c$",RED,14,prefer=[(0.35,-1),(1,-0.4)])
P.solve()

C=Callouts(ax,fig,ren,r_px=11.5,pad=3.5); C.freeze(P.obstacles)
A=C.add
A(O,r"saddle pivot (revolute, $q_3$)",prefer=[(-1,-0.4)])
A(np.array([1.30,0.42]),"boom welded to ground (passive at Stage 1)",prefer=[(-0.6,-1)])
A(O+3.2*h,r"dipper handle (prismatic, $d_4$)",prefer=[(-1,0.7)])
A(E+0.4*h,"dipper",prefer=[(0.9,-0.4)])
A(com,r"COM: $M_d=54{,}300$ kg, $I_{zz}=287{,}900$ kg m$^2$",prefer=[(1,0.4)])
A(O,r"$\{o_2\}\equiv\{o_3\}$ saddle and crowd frames coincide ($a_3=0$)",prefer=[(-1,0.7)])
A(b1,r"$\{o_4\}$ handle-end frame",prefer=[(0.5,1)])
A(W0,r"$\{W\}$ world frame",prefer=[(0,-1)])
miss=C.solve()

axk.set_xlim(0,1); axk.set_ylim(0,1)
axk.text(0.02,0.965,"Key",color=NAVY,fontsize=13.5,fontweight="bold",va="top")
y=0.905
for line in C.key_lines():
    num,rest=line.split("   ",1)
    axk.text(0.02,y,num,color=NAVY,fontsize=10.5,fontweight="bold",va="top")
    axk.text(0.085,y,rest,color="#333333",fontsize=10.5,va="top")
    y-=0.062
axk.text(0.02,y-0.02,r"$r=d_4+c$ is the pivot-to-COM lever arm;"+"\n"+
         r"$c=-1.32$ m is a fixed bookkeeping constant.",color=GREY,fontsize=9.8,va="top")
tt,tg,clip=audit(ax,fig,ren,P.obstacles)
print(f"fig1b | callouts unplaced {len(miss)} {miss} | sym text-text {len(tt)} | sym on geom {len(tg)} | clipped {clip}")

fig.savefig("fig1b_abstraction_300dpi.png",dpi=300,bbox_inches="tight",facecolor="white")
fig.savefig("fig1b_abstraction.pdf",bbox_inches="tight",facecolor="white")
fig.savefig("preview_b.png",dpi=95,bbox_inches="tight",facecolor="white")
