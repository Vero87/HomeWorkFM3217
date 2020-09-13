within ;
package FM3217_2020_VM "tutorial 2"

  package Tutorial1

    model SimplePendulum

    end SimplePendulum;
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
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation
        (Placement(transformation(
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
  annotation (uses(Modelica(version="3.2.3")));
end FM3217_2020_VM;
