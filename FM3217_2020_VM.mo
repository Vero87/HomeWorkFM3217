within ;
package FM3217_2020_VM "tutorial 2"

  package Tutorial1

    model SimplePendulum
     constant Real g(unit="m/s2") = 9.81;
     parameter Real L(min=0, unit="m") = 1;
     Real Theta(start=1, fixed = true, unit="rad");
     Real ThetaDot;
    equation
      der(Theta) = ThetaDot;
      der(ThetaDot) = - g/L * sin(Theta);
    end SimplePendulum;

    model SimplePendulumSIunits
     constant Modelica.SIunits.Acceleration g = 9.81;
     parameter Modelica.SIunits.Length L(min=0) = 1;
     Modelica.SIunits.Angle Theta(start=1, fixed = true);
     Modelica.SIunits.AngularVelocity ThetaDot;
    equation
      der(Theta) = ThetaDot;
      der(ThetaDot) = - g/L * sin(Theta);
    end SimplePendulumSIunits;

    model SimplePendulumSIunitsImport
      import SI = Modelica.SIunits;
      constant SI.Acceleration g=9.81;
      parameter SI.Length L(min=0) = 1;
      SI.Angle Theta(start=1, fixed=true);
      SI.AngularVelocity ThetaDot;
    equation
      der(Theta) = ThetaDot;
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulumSIunitsImport;

    model SimplePendulumDocumentation "Simple Pendulum"
      import SI = Modelica.SIunits;
      constant SI.Acceleration g=9.81 "Gravitational acceleration";
      parameter SI.Length L(min=0) = 1 "Length of the pendulum";
      SI.Angle Theta(start=1, fixed=true)
        "Angle of the pendulum's displacement";
      SI.AngularVelocity ThetaDot;
    equation
      der(Theta) = ThetaDot "Dummy equation";
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulumDocumentation;

    model SimplePendulumEndOfTut1 "Model of a simple pendulum"
      constant Modelica.SIunits.Acceleration g = 9.81 "Gravitational constant";
      parameter Modelica.SIunits.Length L = 1 "Length of the Pendulum";

      // Now come the variables
      Modelica.SIunits.Angle Theta(start=0.1,
       fixed=true) "Angle of the pendulum";
      /* start of comment
  some comment
  */
      Modelica.SIunits.AngularVelocity ThetaDot;
    equation
      ThetaDot = der(Theta);
      der(ThetaDot) = - g/L * sin(Theta);
    end SimplePendulumEndOfTut1;
  end Tutorial1;

  package Tutorial2
    model Motor
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{-8,40},{12,60}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{36,-10},{56,10}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-78,-76},{-58,-56}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-68,0})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia
        annotation (Placement(transformation(extent={{74,-10},{94,10}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                        "Flange of right shaft"
        annotation (Placement(transformation(extent={{92,-10},{112,10}})));
    equation
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-68,10},{
              -68,50},{-60,50}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-40,50},{-8,50}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{56,0},{74,0}}, color={0,0,0}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{12,50},{46,50},{46,10}}, color={0,0,255}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{-68,-10},{-68,-56}}, color={0,0,255}));
      connect(emf.n, ground.p) annotation (Line(points={{46,-10},{48,-10},{48,
              -56},{-68,-56}}, color={0,0,255}));
      connect(signalVoltage.v, u)
        annotation (Line(points={{-80,0},{-120,0}}, color={0,0,127}));
      connect(inertia.flange_b, flange_b1)
        annotation (Line(points={{94,0},{102,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model MotorDrive
      Motor motor(
        resistor(R=0.5),
        inductor(L=0.05),
        inertia(J=0.001))
        annotation (Placement(transformation(extent={{-12,20},{8,40}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-96,20},{-76,40}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-68,20},{-48,40}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{58,20},{78,40}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{22,20},{42,40}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={78,-16})));
      Modelica.Blocks.Continuous.PID PID
        annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
    equation
      connect(feedback.u1, step.y)
        annotation (Line(points={{-66,30},{-75,30}}, color={0,0,127}));
      connect(PID.u, feedback.y)
        annotation (Line(points={{-42,30},{-49,30}}, color={0,0,127}));
      connect(motor.u, PID.y)
        annotation (Line(points={{-14,30},{-19,30}}, color={0,0,127}));
      connect(idealGear.flange_a, motor.flange_b1)
        annotation (Line(points={{22,30},{8.2,30}}, color={0,0,0}));
      connect(inertia.flange_a, idealGear.flange_b)
        annotation (Line(points={{58,30},{42,30}}, color={0,0,0}));
      connect(inertia.flange_b, angleSensor.flange)
        annotation (Line(points={{78,30},{78,-6}}, color={0,0,0}));
      connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-58,22},{
              -58,-27},{78,-27}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MotorDrive;
  end Tutorial2;

  package Tutorial3
    package Components
      model Motor
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
          annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
          annotation (Placement(transformation(extent={{-8,40},{12,60}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{36,-10},{56,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-78,-76},{-58,-56}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-68,0})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{74,-10},{94,10}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                          "Flange of right shaft"
          annotation (Placement(transformation(extent={{92,-10},{112,10}})));
      equation
        connect(signalVoltage.p, resistor.p) annotation (Line(points={{-68,10},{
                -68,50},{-60,50}}, color={0,0,255}));
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-40,50},{-8,50}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{56,0},{74,0}}, color={0,0,0}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{12,50},{46,50},{46,10}}, color={0,0,255}));
        connect(signalVoltage.n, ground.p)
          annotation (Line(points={{-68,-10},{-68,-56}}, color={0,0,255}));
        connect(emf.n, ground.p) annotation (Line(points={{46,-10},{48,-10},{48,
                -56},{-68,-56}}, color={0,0,255}));
        connect(signalVoltage.v, u)
          annotation (Line(points={{-80,0},{-120,0}}, color={0,0,127}));
        connect(inertia.flange_b, flange_b1)
          annotation (Line(points={{94,0},{102,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motor;

      model DCMachine

        parameter Modelica.SIunits.Reactance R = 0.5 "Resistance of the armature";

        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
          annotation (Placement(transformation(extent={{-8,40},{12,60}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{36,-10},{56,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{36,-80},{56,-60}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{74,-10},{94,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                          "Flange of right shaft"
          annotation (Placement(transformation(extent={{92,-10},{112,10}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin"
          annotation (Placement(transformation(extent={{-120,52},{-80,92}}),
              iconTransformation(extent={{-120,52},{-80,92}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin"
          annotation (Placement(transformation(extent={{-124,-86},{-82,-44}}),
              iconTransformation(extent={{-124,-86},{-82,-44}})));
        parameter Modelica.SIunits.Inductance L=0.1 "Inductance of DC Machine";
        parameter Modelica.SIunits.Inertia J=0.1 "Moment of inertia"
          annotation (Dialog(tab="Mechanical"));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-40,50},{-8,50}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{56,0},{74,0}}, color={0,0,0}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{12,50},{46,50},{46,10}}, color={0,0,255}));
        connect(inertia.flange_b, flange_b1)
          annotation (Line(points={{94,0},{102,0}}, color={0,0,0}));
        connect(resistor.p, p) annotation (Line(points={{-60,50},{-80,50},{-80,72},{-100,
                72}}, color={0,0,255}));
        connect(p, p)
          annotation (Line(points={{-100,72},{-100,72}}, color={0,0,255}));
        connect(emf.n, n)
          annotation (Line(points={{46,-10},{46,-65},{-103,-65}}, color={0,0,255}));
        connect(emf.n, ground.p)
          annotation (Line(points={{46,-10},{46,-60}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{96,-90},{-96,136}}, fileName="modelica://FM3217_2020_VM/pictures/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachine;

      model Rload "Resistive load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_load)
          annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={0,0})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
          "Positive electrical pin"
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
        parameter Modelica.SIunits.Resistance R_load=0.5 "Ohmic value of bla ";
      equation
        connect(resistor.p, p1)
          annotation (Line(points={{0,10},{0,100}}, color={0,0,255}));
        connect(resistor.n, n1)
          annotation (Line(points={{0,-10},{0,-100}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RLload
        extends Rload;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-46,0})));
        parameter Modelica.SIunits.Inductance L_load=0.1 "Load inductance";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-46,10},{-46,62},{0,
                62},{0,100}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-46,-10},{-46,-68},{0,
                -68},{0,-100}}, color={0,0,255}));
      end RLload;

      model RLCload "Capasitor"
        extends RLload;
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={42,0})));
        parameter Modelica.SIunits.Capacitance C_load=0.001;
      equation
        connect(capacitor.p, p1) annotation (Line(points={{42,10},{42,62},{0,62},
                {0,100}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{42,-10},{42,-68},{0,
                -68},{0,-100}}, color={0,0,255}));
      end RLCload;

      model Turbine
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t)
          annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=T_t)
          annotation (Placement(transformation(extent={{54,-10},{34,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(rotation=0, extent={{-110,-10},{-90,10}})));
        parameter Modelica.SIunits.Inertia J_t=2 "Turbine inertia";
        parameter Modelica.SIunits.Torque T_t=10 "Turbine torque";
      equation
        connect(inertia.flange_b, constantTorque.flange)
          annotation (Line(points={{-24,0},{34,0}}, color={0,0,0}));
        connect(flange_a, inertia.flange_a)
          annotation (Line(points={{-100,0},{-44,0}}, color={0,0,0}));
        annotation (Icon(graphics={Bitmap(extent={{-98,-98},{98,98}}, fileName=
                    "modelica://FM3217_2020_VM/pictures/Turbine.png")}));
      end Turbine;
    end Components;

    package Tests
      model MotorDrive
        Tutorial2.Motor motor(
          resistor(R=0.5),
          inductor(L=0.05),
          inertia(J=0.001))
          annotation (Placement(transformation(extent={{-12,20},{8,40}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-96,20},{-76,40}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{-68,20},{-48,40}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{58,20},{78,40}})));
        Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
          annotation (Placement(transformation(extent={{22,20},{42,40}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={78,-16})));
        Modelica.Blocks.Continuous.PID PID
          annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
      equation
        connect(feedback.u1, step.y)
          annotation (Line(points={{-66,30},{-75,30}}, color={0,0,127}));
        connect(PID.u, feedback.y)
          annotation (Line(points={{-42,30},{-49,30}}, color={0,0,127}));
        connect(motor.u, PID.y)
          annotation (Line(points={{-14,30},{-19,30}}, color={0,0,127}));
        connect(idealGear.flange_a, motor.flange_b1)
          annotation (Line(points={{22,30},{8.2,30}}, color={0,0,0}));
        connect(inertia.flange_a, idealGear.flange_b)
          annotation (Line(points={{58,30},{42,30}}, color={0,0,0}));
        connect(inertia.flange_b, angleSensor.flange)
          annotation (Line(points={{78,30},{78,-6}}, color={0,0,0}));
        connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-58,22},{
                -58,-27},{78,-27}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end MotorDrive;

      model DCMachineTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{-10,-8},{10,12}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-34,0})));
        Modelica.Blocks.Sources.Step step
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{38,-8},{58,12}})));
      equation
        connect(signalVoltage.p, dCMachine.p) annotation (Line(points={{-34,10},
                {-34,14},{-16,14},{-16,9.2},{-10,9.2}}, color={0,0,255}));
        connect(signalVoltage.n, dCMachine.n) annotation (Line(points={{-34,-10},
                {-20,-10},{-20,-4.5},{-10.3,-4.5}}, color={0,0,255}));
        connect(step.y, signalVoltage.v)
          annotation (Line(points={{-59,0},{-46,0}}, color={0,0,127}));
        connect(dCMachine.flange_b1, inertia.flange_a)
          annotation (Line(points={{10.2,2},{38,2}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Components.RLCload rLCload
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        Components.Turbine turbine annotation (Placement(transformation(
                rotation=0, extent={{32,-10},{52,10}})));
      equation
        connect(dCMachine.flange_b1, turbine.flange_a)
          annotation (Line(points={{10.2,0},{32,0}}, color={0,0,0}));
        connect(rLCload.p1, dCMachine.p) annotation (Line(points={{-60,10},{-60,
                24},{-10,24},{-10,7.2}}, color={0,0,255}));
        connect(rLCload.n1, dCMachine.n) annotation (Line(points={{-60,-10},{
                -60,-22},{-10.3,-22},{-10.3,-6.5}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;
    end Tests;
  end Tutorial3;

  package Tutorial4
    model ElectricKattle
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=sqrt(2)*230)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-80,0})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-90,-62},{-70,-42}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=230^2/2000,
          useHeatPort=true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={0,0})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-34,16},{-14,36}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=-90,
            origin={-46,-2})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{32,-10},{52,10}})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=
            4.18e3*1.7)
        annotation (Placement(transformation(extent={{18,28},{38,48}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor
        thermalConductor(G=5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={28,-34})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{16,56},{36,76}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
        annotation (Placement(transformation(extent={{-72,16},{-52,36}})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{64,-4},{84,16}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T
          =294.15)
        annotation (Placement(transformation(extent={{74,-70},{54,-50}})));
    equation
      connect(sineVoltage.n, resistor.n) annotation (Line(points={{-80,-10},{
              -80,-28},{-1.77636e-15,-28},{-1.77636e-15,-10}}, color={0,0,255}));
      connect(ground.p, resistor.n) annotation (Line(points={{-80,-42},{-80,-28},
              {-1.77636e-15,-28},{-1.77636e-15,-10}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p) annotation (Line(points={{-14,26},{
              1.77636e-15,26},{1.77636e-15,10}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p) annotation (Line(points={{-24,36},{
              -24,48},{0,48},{0,26},{1.77636e-15,26},{1.77636e-15,10}}, color={
              0,0,255}));
      connect(powerSensor.nv, resistor.n) annotation (Line(points={{-24,16},{
              -24,-28},{-1.77636e-15,-28},{-1.77636e-15,-10}}, color={0,0,255}));
      connect(powerSensor.power, mean.u) annotation (Line(points={{-34,15},{-46,
              15},{-46,5.2}}, color={0,0,127}));
      connect(resistor.heatPort, temperatureSensor.port)
        annotation (Line(points={{10,0},{32,0}}, color={191,0,0}));
      connect(heatCapacitor.port, temperatureSensor.port)
        annotation (Line(points={{28,28},{28,0},{32,0}}, color={191,0,0}));
      connect(thermalConductor.port_a, temperatureSensor.port)
        annotation (Line(points={{28,-24},{28,0},{32,0}}, color={191,0,0}));
      connect(sineVoltage.p, switch.p) annotation (Line(points={{-80,10},{-80,
              26},{-72,26}}, color={0,0,255}));
      connect(switch.n, powerSensor.pc)
        annotation (Line(points={{-52,26},{-34,26}}, color={0,0,255}));
      connect(temperatureSensor.T, onOffController.u)
        annotation (Line(points={{52,0},{62,0}}, color={0,0,127}));
      connect(const.y, onOffController.reference)
        annotation (Line(points={{37,66},{62,66},{62,12}}, color={0,0,127}));
      connect(onOffController.y, switch.control) annotation (Line(points={{85,6},
              {88,6},{88,82},{-62,82},{-62,38}}, color={255,0,255}));
      connect(thermalConductor.port_b, fixedTemperature.port) annotation (Line(
            points={{28,-44},{28,-60},{54,-60}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ElectricKattle;

    package CoffeeAndMilk_VM
      extends Modelica.Icons.Package;

      model Coffee "Lumped thermal element storing heat"
        Modelica.SIunits.Temperature T(start = 353.15, fixed = true, displayUnit = "degC") "Temperature of element";
        Modelica.SIunits.TemperatureSlope der_T(start = 0) "Time derivative of temperature (= der(T))";
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation(Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        parameter Modelica.SIunits.SpecificHeatCapacity Ccoffee = 4180 "Specific heat capacity of the coffee.";
        parameter Modelica.SIunits.SpecificHeatCapacity Cmilk = 3770 "Specific heat capacity of the milk.";
        Modelica.SIunits.HeatCapacity C "Heat capacity of the coffee and milk mixture";
        parameter Modelica.SIunits.Volume Vcoffee(displayUnit = "ml") = 0.0002 "Volume of the coffee.";
        parameter Modelica.SIunits.Volume Vmilk(displayUnit = "ml") = 1e-05 "Volume of the added milk.";
        parameter Modelica.SIunits.Time AddTime = 300 "The time at which milk is added.";
        parameter Modelica.SIunits.Temperature MilkTemperature = 278.15 "Temperature of the added milk.";
      equation
        C = if time > AddTime then Ccoffee * Vcoffee * 1000 + Cmilk * Vmilk * 1000 else Ccoffee * Vcoffee * 1000;
        when time > AddTime then
          reinit(T, (Ccoffee * Vcoffee * 1000 * T + Cmilk * Vmilk * 1000 * MilkTemperature) / C);
        end when;
        T = port.T;
        der_T = der(T);
        C*der(T) = port.Q_flow;
        annotation (
          Icon(coordinateSystem(initialScale=0.1, grid={10,10}), graphics={Text(
                origin={70,90},
                lineColor={64,64,64},
                extent={{-70,-30},{70,10}},
                textString="%C"), Bitmap(
                origin={0,0},
                extent={{-100,-100},{100,100}},
                fileName="modelica://FM3217_2020/Resources/Images/coffee.png")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
              Polygon(
                points={{0,67},{-20,63},{-40,57},{-52,43},{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{56,-73},{66,-61},{68,-53},{70,-51},{72,-35},{76,-21},{78,-13},{78,3},{74,15},{66,25},{54,33},{44,41},{36,57},{26,65},{0,67}},
                lineColor={160,160,164},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{42,-77},{40,-77},{30,-79},{20,-81},{18,-81},{10,-81},{2,-77},{-12,-73},{-22,-73},{-30,-71},{-40,-65},{-50,-55},{-56,-43},{-58,-35},{-58,-25},{-60,-13},{-60,-5},{-60,7},{-58,17},{-56,19},{-52,27},{-48,35},{-44,45},{-40,57},{-58,35}},
                lineColor={0,0,0},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-6,-1},{6,-12}},
                lineColor={255,0,0},
                fillColor={191,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{11,13},{50,-25}},
                lineColor={0,0,0},
                textString="T"),
              Line(points={{0,-12},{0,-96}}, color={255,0,0})}),
          Documentation(info="<html>
<p>
This is a generic model for the heat capacity of a material.
No specific geometry is assumed beyond a total volume with
uniform temperature for the entire volume.
Furthermore, it is assumed that the heat capacity
is constant (independent of temperature).
</p>
<p>
The temperature T [Kelvin] of this component is a <b>state</b>.
A default of T = 25 degree Celsius (= SIunits.Conversions.from_degC(25))
is used as start value for initialization.
This usually means that at start of integration the temperature of this
component is 25 degrees Celsius. You may, of course, define a different
temperature as start value for initialization. Alternatively, it is possible
to set parameter <b>steadyStateStart</b> to <b>true</b>. In this case
the additional equation '<b>der</b>(T) = 0' is used during
initialization, i.e., the temperature T is computed in such a way that
the component starts in <b>steady state</b>. This is useful in cases,
where one would like to start simulation in a suitable operating
point without being forced to integrate for a long time to arrive
at this point.
</p>
<p>
Note, that parameter <b>steadyStateStart</b> is not available in
the parameter menu of the simulation window, because its value
is utilized during translation to generate quite different
equations depending on its setting. Therefore, the value of this
parameter can only be changed before translating the model.
</p>
<p>
This component may be used for complicated geometries where
the heat capacity C is determined my measurements. If the component
consists mainly of one type of material, the <b>mass m</b> of the
component may be measured or calculated and multiplied with the
<b>specific heat capacity cp</b> of the component material to
compute C:
</p>
<pre>
   C = cp*m.
   Typical values for cp at 20 degC in J/(kg.K):
      aluminium   896
      concrete    840
      copper      383
      iron        452
      silver      235
      steel       420 ... 500 (V2A)
      wood       2500
</pre>
</html>"));
      end Coffee;

      model CoffeeWithMilkPort "Lumped thermal element storing heat"
        Modelica.SIunits.Temperature T(start = 353.15, fixed = true, displayUnit = "degC") "Temperature of element";
        Modelica.SIunits.Enthalpy H annotation(Dialog(group = "Variables"));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation(Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        parameter Modelica.SIunits.SpecificHeatCapacity Ccoffee = 4180 "Specific heat capacity of the coffee.";
        parameter Modelica.SIunits.SpecificHeatCapacity Cmilk = 3770 "Specific heat capacity of the milk.";
        Modelica.SIunits.HeatCapacity C "Heat capacity of the coffee and milk mixture";
        parameter Modelica.SIunits.Volume Vcoffee(displayUnit = "ml") = 0.0002 "Volume of the coffee.";
        parameter Modelica.SIunits.Temperature MilkTemperature = 278.15 "Temperature of the added milk.";
        Modelica.SIunits.Volume Vmilk(displayUnit = "ml", start = 0, fixed = true) "Volume of the added milk.";
        Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 60}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      equation
        C = Ccoffee * Vcoffee * 1000 + Cmilk * Vmilk * 1000;
        der(Vmilk) = u;
        T = port.T;
        T = H / C;
        der(H) = port.Q_flow + Cmilk*1000*u*MilkTemperature;
        annotation (
          Icon(coordinateSystem(initialScale=0.1, grid={10,10}), graphics={Text(
                origin={70,90},
                lineColor={64,64,64},
                extent={{-70,-30},{70,10}},
                textString="%C"), Bitmap(
                origin={0,0},
                extent={{-100,-100},{100,100}},
                fileName="modelica://FM3217_2020/Resources/Images/coffee.png")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
              Polygon(
                points={{0,67},{-20,63},{-40,57},{-52,43},{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{56,-73},{66,-61},{68,-53},{70,-51},{72,-35},{76,-21},{78,-13},{78,3},{74,15},{66,25},{54,33},{44,41},{36,57},{26,65},{0,67}},
                lineColor={160,160,164},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{42,-77},{40,-77},{30,-79},{20,-81},{18,-81},{10,-81},{2,-77},{-12,-73},{-22,-73},{-30,-71},{-40,-65},{-50,-55},{-56,-43},{-58,-35},{-58,-25},{-60,-13},{-60,-5},{-60,7},{-58,17},{-56,19},{-52,27},{-48,35},{-44,45},{-40,57},{-58,35}},
                lineColor={0,0,0},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-6,-1},{6,-12}},
                lineColor={255,0,0},
                fillColor={191,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{11,13},{50,-25}},
                lineColor={0,0,0},
                textString="T"),
              Line(points={{0,-12},{0,-96}}, color={255,0,0})}),
          Documentation(info="<html>
<p>
This is a generic model for the heat capacity of a material.
No specific geometry is assumed beyond a total volume with
uniform temperature for the entire volume.
Furthermore, it is assumed that the heat capacity
is constant (independent of temperature).
</p>
<p>
The temperature T [Kelvin] of this component is a <b>state</b>.
A default of T = 25 degree Celsius (= SIunits.Conversions.from_degC(25))
is used as start value for initialization.
This usually means that at start of integration the temperature of this
component is 25 degrees Celsius. You may, of course, define a different
temperature as start value for initialization. Alternatively, it is possible
to set parameter <b>steadyStateStart</b> to <b>true</b>. In this case
the additional equation '<b>der</b>(T) = 0' is used during
initialization, i.e., the temperature T is computed in such a way that
the component starts in <b>steady state</b>. This is useful in cases,
where one would like to start simulation in a suitable operating
point without being forced to integrate for a long time to arrive
at this point.
</p>
<p>
Note, that parameter <b>steadyStateStart</b> is not available in
the parameter menu of the simulation window, because its value
is utilized during translation to generate quite different
equations depending on its setting. Therefore, the value of this
parameter can only be changed before translating the model.
</p>
<p>
This component may be used for complicated geometries where
the heat capacity C is determined my measurements. If the component
consists mainly of one type of material, the <b>mass m</b> of the
component may be measured or calculated and multiplied with the
<b>specific heat capacity cp</b> of the component material to
compute C:
</p>
<pre>
   C = cp*m.
   Typical values for cp at 20 degC in J/(kg.K):
      aluminium   896
      concrete    840
      copper      383
      iron        452
      silver      235
      steel       420 ... 500 (V2A)
      wood       2500
</pre>
</html>"));
      end CoffeeWithMilkPort;

      model Milk
        Modelica.Blocks.Sources.Pulse pulse(
          nperiod=1,
          width=100,
          period=AddDuration,
          amplitude=AddedVolume/AddDuration,
          startTime=AddTime) annotation (Placement(visible=true, transformation(
              origin={0,0},
              extent={{-20,-20},{20,20}},
              rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(
            visible=true,
            transformation(
              origin={60,0},
              extent={{-10,-10},{10,10}},
              rotation=0),
            iconTransformation(
              origin={0,-100},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        parameter Modelica.SIunits.Time AddTime=300 "The time at which milk is added.";
        parameter Modelica.SIunits.Time AddDuration=1 "Duration of the adding of milk.";
        parameter Modelica.SIunits.Volume AddedVolume(displayUnit="ml") = 1e-05 "Amount of milk added.";
      equation
        connect(pulse.y, y) annotation (Line(
            visible=true,
            points={{22,0},{60,0}},
            color={1,37,163}));
        annotation (Icon(coordinateSystem(grid={2,2}), graphics={Bitmap(extent={{-80,-100},{100,100}}, fileName="modelica://FM3217_2020/Resources/Images/milk.png")}), Diagram(coordinateSystem(preserveAspectRatio=true, grid={2,2})));
      end Milk;

      package Scenarios
        extends Modelica.Icons.Package;

        model Approach1
          extends Modelica.Icons.Example;
          Coffee coffee(AddTime = AddTime) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
          Modelica.Thermal.HeatTransfer.Components.ThermalConductor mug(G = 0.54) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T = 293.15) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
          parameter Modelica.SIunits.Time AddTime = 300 "The time at which milk is added.";
        equation
          connect(ambient.port, mug.port_b) annotation(Line(visible = true, points={{30,0},{10,0}},       color = {191, 0, 0}));
          connect(coffee.port, mug.port_a) annotation(Line(visible = true, points={{-30,0},{-10,0}},     color = {191, 0, 0}));
          annotation(experiment(StopTime = 500, __Wolfram_SynchronizeWithRealTime = false), __Wolfram(PlotSet(plots = {Plot(name = "CoffeeTemperature", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = coffee.T, legend = "Coffee Temperature")}), SubPlot(curves = {Curve(x = time, y = coffee.port.Q_flow, legend = "Heat Flow")})}), Plot(name = "CoffeeHeatCapacity", subPlots = {SubPlot(curves = {Curve(x = time, y = coffee.C, legend = "Heat Capacity")})})})), Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
        end Approach1;

        model Approach2
          extends Modelica.Icons.Example;
          CoffeeWithMilkPort coffee annotation(Placement(visible = true, transformation(origin = {-40, -0}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
          Modelica.Thermal.HeatTransfer.Components.ThermalConductor mug(G = 0.54) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T = 293.15) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
          Milk milk(AddDuration = 1, AddTime=10)   annotation(Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(mug.port_b, ambient.port) annotation (
            Line(points = {{10, 0}, {30, 0}, {30, 0}, {30, 0}}, color = {191, 0, 0}));
          connect(milk.y, coffee.u) annotation (
            Line(points={{-40,20},{-40,20},{-40,6},{-40,6}},          color = {0, 0, 127}));
          connect(coffee.port, mug.port_a) annotation (
            Line(points = {{-30, 0}, {-10, 0}, {-10, 0}, {-10, 0}}, color = {191, 0, 0}));
          connect(ambient.port, mug.port_b) annotation(Line(visible = true, points={{30,0},{10,0}},       color = {191, 0, 0}));
          connect(coffee.port, mug.port_a) annotation(Line(visible = true, points={{-30,0},{-10,0}},      color = {191, 0, 0}));
          connect(milk.y, coffee.u) annotation(Line(visible = true, points={{-40,20},{-40,6},{-40,6}},                          color = {1, 37, 163}));
          annotation(__Wolfram(PlotSet(plots = {Plot(name = "CoffeeTemperature", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = coffee.port.T, legend = "Coffee Temperature")}), SubPlot(curves = {Curve(x = time, y = coffee.port.Q_flow, legend = "Heat Flow")})}), Plot(name = "CoffeeHeatCapacity", subPlots = {SubPlot(curves = {Curve(x = time, y = coffee.C, legend = "Heat Capacity")})})})), experiment(StopTime = 500, StartTime = 0, Tolerance = 1e-6, Interval = 1), Diagram(coordinateSystem(extent = {{-80, -40}, {80, 60}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
        end Approach2;
        annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
      end Scenarios;

      annotation (                                Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
    end CoffeeAndMilk_VM;

    package CupOfCoffee
      "Animation of the refilling process of a cup of coffee."

      model CupOfCoffee_1
        Real T(start=380);
      equation
        der(T) = -0.2*(T - 300);
      end CupOfCoffee_1;

      model CupOfCoffee_2 "Add some text"
        Real T(start=380) "Coffee Temperature";
      equation
        der(T) = -0.2*(T - 300) "Newton's Law of Cooling";
      end CupOfCoffee_2;

      model CupOfCoffee_3 "Use parameters"
        parameter Real T0=380 "Initial temp.";
        parameter Real Tamb=300 "Ambient temperature";
        parameter Real C=0.2;
        Real T(start=T0) "Coffee Temperature";
      equation
        der(T) = -C*(T - Tamb) "Newton's Law of Cooling";
      end CupOfCoffee_3;

      model CupOfCoffee_4 "More physical"
        import Modelica.SIunits.*;
        parameter Temperature T0=380 "Initial temp.";
        parameter Temperature Tamb=300 "Ambient temperature";
        parameter Density rho=1000 "Coffee density";
        parameter SpecificHeatCapacity cv=4179 "Coffee specific heat";
        parameter CoefficientOfHeatTransfer h=25 "Convection coefficient";
        parameter Volume V=4e-4 "Volume of coffee";
        parameter Area A=4e-3 "Area of coffee";
        Temperature T(start=T0) "Coffee Temperature";
      equation
        rho*V*cv*der(T) = -h*A*(T - Tamb) "First law of thermodynamics";
      end CupOfCoffee_4;

      model CupOfCoffee_5
        ThermalCapacitance coffee(
          T0=380,
          rho=1000,
          V=4e-4,
          cv=4179);
        Convection cooling(
          h=25,
          A=4e-3,
          Tamb=300);
      equation
        connect(coffee.p, cooling.p);
      end CupOfCoffee_5;

      model CupOfCoffee_6
        ThermalCapacitance coffee(
          T0=380,
          rho=1000,
          V=4.08e-4,
          cv=4179);
        ThermalCapacitance cup(
          T0=300,
          rho=3700,
          V=8.45e-5,
          cv=880);
        Boundary cup2coffee(h=100, A=2.53e-2);
        Convection coffee_cooling(
          h=25,
          A=4e-3,
          Tamb=300);
        Convection cup_cooling(
          h=25,
          A=2.79e-2,
          Tamb=300);
      equation
        connect(coffee.p, cup2coffee.p1);
        connect(coffee.p, coffee_cooling.p);
        connect(cup.p, cup2coffee.p2);
        connect(cup.p, cup_cooling.p);
      end CupOfCoffee_6;

      model CupOfCoffee_7 "Additional physics"
        import Modelica.SIunits.*;
        import Modelica.Constants.*;
        parameter CoefficientOfHeatTransfer h_air=25
          "Convection coefficient with air";
        parameter CoefficientOfHeatTransfer h_fluid=100
          "Convection coefficient with fluid";
        parameter Customer customer;
        parameter Service service;
        parameter CoffeeProperties coffee;
        parameter CupProperties cup;
        Temperature T "Coffee Temperature";
        Temperature Tcup "Coffee cup temperature";
        Mass M "Mass of coffee in the cup";
        Area A "Surface area exposed to ambient";
        Length H "Height of coffee in the cup";
        Volume V "Volume of coffee in cup";
        Boolean drinking "true when drinking";
        Boolean empty "true when cup is empty";
        MassFlowRate mdot_drink "drinking mass flow rate";
        discrete MassFlowRate mdot_refill "refilling mass flow rate";
        Energy U "coffee internal energy";
        Area Acup_int "internal surface area of cup";
      initial equation
        H = 0.9*cup.H;
        T = service.Tcoffee;
        Tcup = service.Tamb;
      algorithm
        when sample(service.start_refill, service.dt_refill) then
          mdot_refill := service.dm_refill;
        end when;
        when H>=0.9*cup.H then
          mdot_refill := 0;
        end when;
      equation
        U = M*coffee.cv*T;
        der(U) = -h_air*A*(T - service.Tamb) - h_fluid*Acup_int*(T - Tcup) - mdot_drink*coffee.cp*
          T + mdot_refill*coffee.cp*service.Tcoffee
          "First law of thermodynamics for coffee";
        cup.M*cup.cv*der(Tcup) = -h_air*cup.A_ext*(Tcup - service.Tamb) - h_fluid*Acup_int*(
          Tcup - T) "First law of thermodynamics for cup";
        A = pi*cup.D^2/4 "Area of coffee exposed to air";
        Acup_int = pi*cup.D*(cup.H-H) "Area on inside of mug exposed to air";
        V = A*H "Volume of coffee in the cup";
        M = coffee.rho*V "Mass of coffee in the cup";
        der(M) = -mdot_drink + mdot_refill "Conservation of mass for coffee";
        empty = M <= 1e-9;
        drinking = mod(time, customer.dt_drink) <= customer.drink_duration and not empty;
        mdot_drink = if drinking then customer.sip_rate else 0;
      end CupOfCoffee_7;

      connector ThermalPort
        Modelica.SIunits.Temperature T;
        flow Modelica.SIunits.HeatFlowRate q;
      end ThermalPort;

      model ThermalCapacitance
        import Modelica.SIunits.*;
        parameter Temperature T0;
        parameter Density rho;
        parameter SpecificHeatCapacity cv;
        parameter Volume V;
        ThermalPort p;
      initial equation
        p.T = T0;
      equation
        rho*V*cv*der(p.T) = p.q;
      end ThermalCapacitance;

      model Convection "Convection to the ambient"
        import Modelica.SIunits.*;
        parameter CoefficientOfHeatTransfer h;
        parameter Temperature Tamb;
        parameter Area A;
        ThermalPort p;
      equation
        p.q = h*A*(p.T - Tamb);
      end Convection;

      model Boundary "Convective boundary"
        import Modelica.SIunits.*;
        parameter CoefficientOfHeatTransfer h;
        parameter Area A;
        ThermalPort p1;
        ThermalPort p2;
      equation
        p1.q + p2.q = 0 "No storage of energy";
        p1.q = h*A*(p1.T - p2.T);
      end Boundary;

      record Customer
        import Modelica.SIunits.*;
        Time dt_drink=60 "amount of time between sips";
        Mass dm_drink=1.48e-2 "amount of mass consumed during each sip";
        Time drink_duration=1 "duration for each drink";
        MassFlowRate sip_rate=dm_drink/drink_duration "Rate of drinking coffee";
        Time start_drink=1 "time when drinking starts";
      end Customer;

      record Service
        import Modelica.SIunits.*;
        Time start_refill=1750 "time when refilling starts";
        Time dt_refill=1750 "amount of time between refills";
        Mass dm_refill=0.3 "amount of mass added during each refill";
        Time refill_duration=20 "duration for each refill";
        Temperature Tcoffee=360 "Temperature of refill coffee";
        parameter Temperature Tamb=300 "Ambient temperature";
      end Service;

      record CoffeeProperties
        import Modelica.SIunits.*;
        parameter Density rho=1000 "Coffee density";
        parameter SpecificHeatCapacity cv=4179
          "Coffee specific heat (constant volume)";
        parameter SpecificHeatCapacity cp=cv
          "Coffee specific heat (constant pressure)";
      end CoffeeProperties;

      record CupProperties
        import Modelica.SIunits.*;
        import Modelica.Constants.*;
        Height H=0.127 "Height of coffee cup";
        Diameter D=0.0508 "Diameter of cup base";
        Density rho=3700 "density of the cup";
        Mass M=rho*V "mass of the cup";
        SpecificHeatCapacity cv=880 "Cup specific heat (constant volume)";
        Length t=0.003175 "cup wall thickness";
        Volume V=pi*t*H*(D + t)+pi*(D/2)^2*t "volume of the cup";
        Area A_ext=pi*H*(D + 2*t) "external surface area of cup";
      end CupProperties;

      model CupOfCoffeeAnimation "animated model of cup of coffee"
        CupOfCoffee.CupOfCoffee_7 coffee_example;
        Modelica.SIunits.Temperature coffeeTemp;
        Modelica.SIunits.Temperature mugTemp;
        Modelica.SIunits.Height h;
        function ComputeCoffeeColor
          input Modelica.SIunits.Temperature T;
          output Real color[3];
        protected
          Modelica.SIunits.Temperature Tmin=293;
          Modelica.SIunits.Temperature Tmax=380;
          Real per=(min(max(Tmin, T), Tmax) - Tmin)/(Tmax - Tmin);
        algorithm
          color := {100 + 155*per,100 - 50*per,100 - 50*per};
        end ComputeCoffeeColor;

        function ComputeMugColor
          input Modelica.SIunits.Temperature T;
          output Real color[3];
        protected
          Modelica.SIunits.Temperature Tmin=293;
          Modelica.SIunits.Temperature Tmax=380;
          Real per=(min(max(Tmin, T), Tmax) - Tmin)/(Tmax - Tmin);
        algorithm
          color := {100 + 155*per,100 - 50*per,100 - 50*per};
        end ComputeMugColor;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape coffee(
          shapeType="cylinder",
          length=h,
          width=1.0,
          color=ComputeCoffeeColor(coffeeTemp),
          height=1.0,
          lengthDirection={0,1,0}) annotation (Placement(transformation(extent={{
                  -20,20},{0,40}}, rotation=0)));
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape cup_bottom(
          shapeType="cylinder",
          length=-0.2,
          width=1.2,
          height=1.2,
          lengthDirection={0,1,0},
          color=ComputeMugColor(mugTemp));
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape cup_wall(
          shapeType="pipe",
          length=1.2,
          width=1.2,
          height=1.2,
          lengthDirection={0,1,0},
          extra=0.8,
          color=ComputeMugColor(mugTemp));
      equation
        coffeeTemp = coffee_example.T;
        mugTemp = coffee_example.Tcup;
        h=coffee_example.H/coffee_example.cup.H/0.9;
        annotation (uses);
      end CupOfCoffeeAnimation;

      model TestAll
        CupOfCoffee_1 cup1;
        CupOfCoffee_2 cup2;
        CupOfCoffee_3 cup3;
        CupOfCoffee_4 cup4;
        CupOfCoffee_5 cup5;
        CupOfCoffee_6 cup6;
        CupOfCoffee_7 cup7;
      end TestAll;
      annotation (
        conversion(noneFromVersion="", noneFromVersion="1"));
    end CupOfCoffee;
  end Tutorial4;
  annotation (uses(Modelica(version="3.2.3")));
end FM3217_2020_VM;
