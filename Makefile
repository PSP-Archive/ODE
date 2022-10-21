TARGET = ODEtest
OBJS = ODEtest.o ode/array.o ode/collision_kernel.o ode/collision_quadtreespace.o ode/collision_space.o ode/collision_std.o ode/collision_transform.o ode/collision_util.o ode/error.o ode/export-dif.o ode/fastdot.o ode/fastldlt.o ode/fastlsolve.o ode/fastltsolve.o ode/joint.o ode/lcp.o ode/mass.o ode/mat.o ode/matrix.o ode/memory.o ode/misc.o ode/obstack.o ode/ode.o ode/odemath.o ode/quickstep.o ode/rotation.o ode/step.o ode/stepfast.o ode/testing.o ode/timer.o ode/util.o  brickTex.o metalRibbed.o metalRibbedMedium.o

INCDIR = 

CFLAGS = -O2 -G8 -Wall
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)

LIBDIR = 
LDFLAGS =
LIBS= -lstdc++ -lpspgum -lpspgu -lm

EXTRA_TARGETS = EBOOT.PBP
PSP_EBOOT_TITLE = ODE Test
PSP_EBOOT_ICON = "ODE.png"

PSPSDK=$(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak

logo.o: logo.raw
	bin2o -i logo.raw logo.o logo
	bin2o -i brickTex.raw brickTex.o brickTex
	bin2o -i metalRibbed.raw metalRibbed.o metalRibbed
	bin2o -i metalRibbedMedium.raw metalRibbedMedium.o metalRibbedMedium
