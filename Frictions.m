function F = Friction_Torues_Vector_function( q1dot, q2dot, q3dot, q4dot, q5dot, q6dot)


% Friction_Torues_Vector_function: Computes the Torques Vector due to Coulomb friction, viscous friction, as well as offset torques for given joint velocities.


% The identified base parameters (\vartheta_{b} vector)
coefficients = [

     1.127192704159674e+01
    -7.592139176521678e+00
    -1.214059817341892e+00
    -5.802166487214266e+00
     5.326906738275686e+00
    -1.361027413566556e+01
     6.448609461690413e+01
     1.000018422040135e+01
     7.726490421282266e+00
     2.811937241823886e+00
    -2.028501337197468e+00
     1.080859203473093e+00
    -4.620877101805478e+00
     1.521413209726164e+00
     1.436069115753074e+01
     4.969966618017289e+00
     5.708555923394439e+00
    -1.356794235898829e+00
     4.796481101274910e+00
    -3.755739111306564e+00
     5.274663381701949e+00
     2.041183206656774e-01
     3.434816194586145e+00
    -8.818793425095958e-01
    -6.693261719670063e-01
    -7.172163600458437e-01
    -3.374360711515359e-01
    -2.327540766551572e+00
     5.892564113829182e+00
    -2.656138277067310e+00
     4.166221408188511e+00
    -4.412954965588797e+00
    -4.554881504846105e+00
    -8.160293608153412e-01
     3.910889217987887e+00
    -2.078211662299954e+00
     7.259100472428752e+01
     6.635812473376058e+01
     2.135646068362948e+01
     1.644223863282695e+02
     1.248187739562997e-03
     1.350426695412986e+02
     6.753911572188992e-04
     1.176898085971351e+01
     8.680457573418034e-03
     1.950457462423874e+01
     1.556159281879819e-02
     9.964842235055412e-03
    -1.321403959066689e+01
    -2.943904537748044e+01
    -2.505850231773448e+02
    -1.178494286011038e+01
    -1.632120872955739e+01
    -1.700949877698571e+01];


F1 = coefficients(37)*q1dot + coefficients(38)*sign(q1dot) + coefficients(49);

F2 = coefficients(39)*q2dot + coefficients(40)*sign(q2dot) + coefficients(50);

F3 = coefficients(41) * q3dot + coefficients(42) * sign(q3dot) + coefficients(51);

F4 = coefficients(43) * q4dot + coefficients(44) * sign(q4dot) + coefficients(52);

F5 = coefficients(45) * q5dot + coefficients(46) * sign(q5dot) + coefficients(53);

F6 = coefficients(47) * q6dot + coefficients(48) * sign(q6dot) + coefficients(54);




F = [F1; F2; F3; F4; F5; F6];


% Mass*[q1dotdot;q2dotdot;q3dotdot;q4dotdot;q5dotdot;q6dotdot]+V+G+F




end
