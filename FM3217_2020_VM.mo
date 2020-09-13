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
  annotation (uses(Modelica(version="3.2.3")));
end FM3217_2020_VM;
