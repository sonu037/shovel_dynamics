"""
Figure 1(a) - Cable shovel anatomy, side elevation (schematic, not to scale).
Geometry in CONFIG; labels placed by the shapely engine in layout.py.
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle, FancyBboxPatch
from layout import Placer, Callouts, audit

NAVY="#1F3864"; OCHRE="#C8A15A"; OCHRE_D="#8A6D2F"; RED="#B03A2E"
GREY="#8A8A8A"; PALE="#E4EAF3"; STEEL="#5B739B"; MUTE="#B9C4D4"; SAND="#EFE7D8"
DIP_F="#FFFFFF"; DIP_E="#1A1A1A"                 # dipper: white body, dark edge
F2="#0F7B6C"; F3="#7B3FA0"; F4="#C2185B"; FW="#555555"   # frames {o2} {o3} {o4} {W}
BOOM_ANG=36.0; HANDLE_ANG=-20.0; SADDLE_FRAC=0.60; L_BOOM=8.2
plt.rcParams.update({"font.family":"serif","mathtext.fontset":"cm"})

U=lambda d:(lambda t:np.array([np.cos(t),np.sin(t)]))(np.radians(d))
N=lambda v:np.array([-v[1],v[0]])
def beam(ax,p,q,w,fc,ec,lw=1.6,z=5):
    d=(q-p)/np.linalg.norm(q-p); nn=N(d)
    ax.add_patch(Polygon([p+w*nn,q+w*nn,q-w*nn,p-w*nn],closed=True,fc=fc,ec=ec,lw=lw,zorder=z))

def curved_dipper(ax,E,h,hn,c_edge,c_face,z=9):
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    P=lambda a,b: E+a*h+b*hn
    verts=[P(0.94,0.56), P(0.94,-0.32), P(0.74,-0.70), P(0.10,-0.76),
           P(-0.36,-0.46), P(-0.36,0.38), P(-0.12,0.60), P(0.94,0.56)]
    codes=[Path.MOVETO,Path.LINETO,Path.CURVE3,Path.CURVE3,
           Path.CURVE3,Path.LINETO,Path.CURVE3,Path.CURVE3]
    ax.add_patch(PathPatch(Path(verts,codes),fc=c_face,ec=c_edge,lw=1.7,zorder=z))

def curved_link(ax,p,q,bulge,color,lw=2.0,z=10):
    """Bail link drawn as a shallow curve - a real bail is not a sharp bend."""
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    d=q-p; m=(p+q)/2+bulge*N(d/np.linalg.norm(d))
    ax.add_patch(PathPatch(Path([p,m,q],[Path.MOVETO,Path.CURVE3,Path.CURVE3]),
                           fc="none",ec=color,lw=lw,zorder=z,capstyle="round"))

def tang(P,C,R):
    v=P-C; L2=v@v; e=np.sqrt(max(L2-R*R,1e-9)); b=C+(R*R/L2)*v; o=(R*e/L2)*N(v); return b+o,b-o
def pulley(ax,C,R,c,z=9):
    ax.add_patch(Circle(C,R,fc="#FFFFFF",ec=c,lw=2.0,zorder=z))
    ax.add_patch(Circle(C,R*0.78,fc="none",ec=c,lw=0.9,ls=(0,(3,2)),zorder=z+1))
    for k in range(6):
        t=np.radians(30+60*k); d=np.array([np.cos(t),np.sin(t)])
        ax.plot(*zip(C+0.22*R*d,C+0.72*R*d),color=c,lw=0.8,zorder=z+1)
    ax.add_patch(Circle(C,R*0.20,fc=c,zorder=z+2))

fig=plt.figure(figsize=(16.8,9.2))
gs=fig.add_gridspec(1,2,width_ratios=[1.0,0.30],wspace=0.01)
ax=fig.add_subplot(gs[0]); axk=fig.add_subplot(gs[1])
ax.set_aspect("equal"); ax.axis("off"); axk.axis("off")
u=U(BOOM_ANG); n=N(u); h=U(HANDLE_ANG); hn=N(h)

ax.plot([-4.0,17.0],[0.05,0.05],color=GREY,lw=1.4,zorder=1)
ax.add_patch(FancyBboxPatch((-0.15,0.16),4.5,0.92,boxstyle="round,pad=0.08,rounding_size=0.32",
                            fc="#4C4C4C",ec="#2F2F2F",lw=1.1,zorder=2))
for x in np.linspace(0.35,3.85,5):
    ax.add_patch(Circle((x,0.62),0.28,fc="#2B2B2B",ec="#171717",lw=0.9,zorder=3))
ax.add_patch(Rectangle((0.20,1.08),3.95,2.00,fc=PALE,ec=STEEL,lw=1.5,zorder=3))
ax.add_patch(Rectangle((0.58,3.08),1.85,0.70,fc="#FFFFFF",ec=STEEL,lw=1.2,zorder=4))
gtop=np.array([2.15,4.85])
for fx in (0.85,3.55): ax.plot([fx,gtop[0]],[3.78,gtop[1]],color=STEEL,lw=2.1,zorder=4)
ax.plot([1.20,3.20],[4.24,4.24],color=STEEL,lw=1.1,zorder=4)
drum=np.array([2.80,2.20]); ax.add_patch(Circle(drum,0.30,fc="#FFFFFF",ec=STEEL,lw=1.5,zorder=5))
ax.add_patch(Circle(drum,0.09,fc=STEEL,zorder=6))

foot=np.array([4.10,1.30]); tip=foot+L_BOOM*u
beam(ax,foot,tip,0.27,MUTE,STEEL,1.4,z=4)
sheave=tip.copy(); R_SH=0.42; pulley(ax,sheave,R_SH,NAVY,z=9)
brk=tip-0.85*u+0.27*n
ax.add_patch(Polygon([brk+0.34*u+0.17*n,brk-0.34*u+0.17*n,brk-0.34*u-0.10*n,brk+0.34*u-0.10*n],
                     closed=True,fc="#FFFFFF",ec=STEEL,lw=1.5,zorder=8))
for k in (-0.17,0.17): ax.add_patch(Circle(brk+k*u+0.04*n,0.062,fc=STEEL,zorder=9))
for dy in (0.09,-0.09):
    ax.plot([gtop[0],brk[0]],[gtop[1]+dy,brk[1]+dy*0.3],color=STEEL,lw=1.3,ls=(0,(7,3)),zorder=5)

sad=foot+SADDLE_FRAC*L_BOOM*u
a0=sad-2.35*h; b0=sad+5.30*h
beam(ax,a0,b0,0.20,OCHRE,OCHRE_D,1.5,z=9)
ax.add_patch(Polygon([sad+0.48*h+0.42*hn,sad+0.48*h-0.42*hn,sad-0.48*h-0.42*hn,sad-0.48*h+0.42*hn],
                     closed=True,fc="#FFFFFF",ec=NAVY,lw=1.8,zorder=10))
ax.add_patch(Circle(sad,0.115,fc=RED,zorder=11))
E=b0+0.20*h
curved_dipper(ax,E,h,hn,DIP_E,DIP_F,z=9)
for s_ in np.linspace(-0.52,0.40,4):
    p0=E+0.90*h+s_*hn; ax.plot(*zip(p0,p0+0.26*h),color=NAVY,lw=1.9,zorder=10)
# The bail is a two-force link: pinned at the dipper lugs, loaded only at the rope
# socket, so under tension it aligns with the rope. Its direction therefore depends on
# where the rope leaves the sheave, and the rope's tangent point depends on the socket
# position - so solve the pair by fixed-point iteration instead of placing it by eye.
lug1=E+0.80*h+0.50*hn; lug2=E-0.28*h+0.50*hn
lugmid=(lug1+lug2)/2; L_BAIL=0.95
bail=lugmid+L_BAIL*hn                       # starting guess
for _ in range(30):
    T=tang(bail,sheave,R_SH)[0]             # where the rope leaves the sheave
    e_rope=(T-lugmid)/np.linalg.norm(T-lugmid)
    new_bail=lugmid+L_BAIL*e_rope           # link swings onto the load line
    if np.linalg.norm(new_bail-bail)<1e-9: bail=new_bail; break
    bail=new_bail
BAIL_TILT=np.degrees(np.arctan2(*(bail-lugmid)[::-1]))
for lug,bg in ((lug1,-0.06),(lug2,0.06)):
    curved_link(ax,lug,bail,bg,NAVY,2.0,z=10)
    ax.add_patch(Circle(lug,0.075,fc="#FFFFFF",ec=NAVY,lw=1.3,zorder=11))
ax.add_patch(Circle(bail,0.115,fc="#FFFFFF",ec=NAVY,lw=1.8,zorder=12))
ax.add_patch(Circle(bail,0.045,fc=NAVY,zorder=13))

Ta=tang(drum,sheave,R_SH)[1]; Tb=tang(bail,sheave,R_SH)[0]
ax.plot(*zip(drum,Ta),color=NAVY,lw=1.6,zorder=7)
ax.plot(*zip(bail,Tb),color=NAVY,lw=1.6,zorder=7)
aa=np.arctan2(*(Ta-sheave)[::-1]); bb_=np.arctan2(*(Tb-sheave)[::-1])
if bb_<aa: bb_+=2*np.pi
if bb_-aa>np.pi: aa,bb_=bb_,aa+2*np.pi
th=np.linspace(aa,bb_,80)
ax.plot(*(sheave+R_SH*np.c_[np.cos(th),np.sin(th)]).T,color=NAVY,lw=1.6,zorder=7)
# alpha_3: the angle at the rope socket between the hoist rope and the handle axis.
# The handle axis is projected through the socket (dashed, both ways) so the angle is
# read the way it is defined in the source, not against an arbitrary construction line.
rd=(Tb-bail)/np.linalg.norm(Tb-bail)
ax.plot(*zip(bail-1.45*h,bail+0.55*h),color=GREY,lw=0.9,ls=(0,(5,4)),zorder=6)
ar=np.arctan2(rd[1],rd[0]); ah=np.arctan2(-h[1],-h[0])
d=(ar-ah+np.pi)%(2*np.pi)-np.pi
tha=np.linspace(ah,ah+d,60); arc_a=bail+0.95*np.c_[np.cos(tha),np.sin(tha)]
ax.plot(arc_a[:,0],arc_a[:,1],color=RED,lw=1.4,zorder=12)
ax.add_patch(Polygon([(13.1,0.05),(15.6,0.05),(15.6,3.7),(14.5,2.9),(13.9,1.6),(13.1,0.9)],
                     closed=True,fc=SAND,ec="#C9BCA3",lw=1.0,zorder=0))

# subsystem emphasis: everything modelled at Stage 1 is drawn in ochre/navy above
ax.text(-3.9,9.15,"(a)  Machine anatomy",color=NAVY,fontsize=16,fontweight="bold")
ax.text(-3.9,8.62,"the Stage-1 model covers the saddle pivot, the dipper handle and the dipper (coloured);"
                  "\nboom, gantry, ropes and swing are shown for context and enter at later milestones",
        color=GREY,fontsize=11.2,va="top")
ax.set_xlim(-4.2,17.4); ax.set_ylim(-2.8,9.6)

fig.canvas.draw(); ren=fig.canvas.get_renderer()
probe=Placer(ax,fig,ren); probe.freeze_obstacles()

C=Callouts(ax,fig,ren,r_px=11.5,pad=3.5); C.freeze(probe.obstacles)
A=C.add
A(np.array([1.6,0.62]),"crawler tracks (lower works)",prefer=[(-0.7,-1)])
A(np.array([2.0,2.0]),"house / upper works (swing frame)",prefer=[(-1,0.2)])
A(gtop,"gantry (A-frame)",prefer=[(-1,0.6)])
A((gtop+brk)/2,"suspension (pendant) ropes",prefer=[(0,1)])
A(brk,"boom-head connector",prefer=[(0.4,1)])
A(drum,"hoist drum",prefer=[(-1,-0.4)])
A(drum+0.42*(Ta-drum),"hoist rope",prefer=[(-0.5,1),(0,1)])
A(sheave,"boom-point sheave",prefer=[(0.5,1)])
A(foot,"boom foot (pinned to the house)",prefer=[(-0.3,-1)])
A(foot+0.42*L_BOOM*u,"boom",prefer=[(0.4,1)])
A(sad,"saddle block (revolute pivot, $q_3$)",prefer=[(-0.5,-1)])
A(a0+0.9*h,"dipper handle (prismatic crowd, $d_4$)",prefer=[(-1,-0.4)])
A(bail,"dipper bail (yoke and rope socket)",prefer=[(1,0.3)])
A(lug1,"bail pins in the dipper lugs",prefer=[(1,-0.4)])
A(arc_a[len(arc_a)//2],r"$\alpha_3$ (rope angle at the bail)",prefer=[(-1,-0.3)])
A(E+0.5*h,"dipper",prefer=[(0.5,-1)])
A(E+1.10*h-0.06*hn,"dipper teeth (bank interface, where the digging force acts)",prefer=[(0.6,-1),(1,-0.3)])
miss=C.solve()

axk.set_xlim(0,1); axk.set_ylim(0,1)
axk.text(0.02,0.965,"Key",color=NAVY,fontsize=13.5,fontweight="bold",va="top")
y=0.905
for line in C.key_lines():
    num,rest=line.split("   ",1)
    axk.text(0.02,y,num,color=NAVY,fontsize=10.5,fontweight="bold",va="top")
    axk.text(0.11,y,rest,color="#333333",fontsize=10.5,va="top")
    y-=0.049
axk.text(0.02,y-0.02,"Coloured members are modelled at Stage 1;\ngreyed members enter at later milestones.",
         color=GREY,fontsize=9.6,va="top")
print("fig1a | callouts unplaced:",len(miss),miss)

fig.savefig("fig1a_machine_300dpi.png",dpi=300,bbox_inches="tight",facecolor="white")
fig.savefig("fig1a_machine.pdf",bbox_inches="tight",facecolor="white")
fig.savefig("preview_a.png",dpi=95,bbox_inches="tight",facecolor="white")
