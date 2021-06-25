module PROWIMData
data = Dict()

data["clalpha0"] = [
9.72762645914397 0.02932960893854747;
17.509727626459146 0.032681564245810035;
25.29182879377432 0.04273743016759776;
28.404669260700395 0.04273743016759776;
31.51750972762646 0.09636871508379888;
34.24124513618677 0.08966480446927375;
37.7431906614786 0.0812849162011173;
40.85603112840467 0.05111731843575418;
52.918287937743195 -0.02597765363128493;
56.03112840466927 -0.05782122905027934;
59.14396887159533 -0.07793296089385479;
62.2568093385214 -0.08296089385474861;
65.36964980544747 -0.04608938547486033;
68.48249027237355 -0.020949720670391053;
74.70817120622567 -0.019273743016759798;
80.93385214007782 -0.01424581005586592;
87.15953307392996 -0.010893854748603354;
91.82879377431905 -0.00754189944134078;
]

data["clalpha4"] = [
    9.737827715355792 0.36797752808988776;
    17.228464419475642 0.37078651685393266;
    25.093632958801493 0.38202247191011246;
    28.089887640449433 0.3792134831460675;
    31.460674157303373 0.43258426966292146;
    34.45692883895131 0.4241573033707866;
    37.453183520599225 0.41292134831460686;
    40.823970037453165 0.34831460674157316;
    52.808988764044926 0.2500000000000001;
    56.179775280898866 0.24719101123595516;
    59.17602996254681 0.23595505617977536;
    62.17228464419475 0.22752808988764056;
    65.54307116104869 0.27247191011235966;
    68.53932584269663 0.2640449438202248;
    74.90636704119848 0.2500000000000001;
    81.27340823970036 0.23033707865168546;
    87.26591760299624 0.19943820224719117;
    92.13483146067418 0.16292134831460686
]

data["clalpha8"] = [
    9.701492537313438 0.6500000000000001;
    17.537313432835827 0.6590909090909094;
    25.000000000000004 0.6636363636363638;
    27.98507462686567 0.6545454545454548;
    31.34328358208956 0.7181818181818183;
    34.32835820895524 0.7227272727272729;
    37.68656716417911 0.7181818181818183;
    40.67164179104479 0.6409090909090913;
    52.98507462686568 0.5227272727272729;
    55.97014925373135 0.5454545454545456;
    59.32835820895524 0.5363636363636366;
    62.313432835820905 0.5227272727272729;
    65.29850746268657 0.5590909090909093;
    68.65671641791046 0.5181818181818183;
    74.62686567164181 0.4818181818181819;
    80.97014925373136 0.44545454545454555;
    87.31343283582092 0.3909090909090911;
    91.79104477611942 0.331818181818182
]

data["CLalpha_propoff"] = [
    0 0;
    1.079545454545455 0.07431693989071053;
    2.121212121212121 0.14644808743169413;
    3.1818181818181825 0.2163934426229509;
    4.242424242424243 0.2863387978142078;
    5.303030303030304 0.35409836065573785;
    6.344696969696968 0.41530054644808756;
    7.386363636363636 0.47650273224043715;
    8.446969696969695 0.5420765027322405;
    9.469696969696969 0.6032786885245902
]

data["CLalpha_propon"] = [
    0 0;
    2.007575757575758 0.1661202185792351;
    4.015151515151515 0.3256830601092897;
    6.0227272727272725 0.47650273224043715;
    7.992424242424243 0.6229508196721312;
    10 0.7693989071038252
]


data["span"] = 640e-3 * 2
data["diameter"] = 236e-3
data["blades"] = 4
data["rotor_y"] = 300e-3
data["chord"] = 240e-3
data["rotor_x"] = 201.8e-3

data["rotor_radii"] = [0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* data["diameter"] / 2
data["rotor_twist"] = [35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0] * pi/180
data["rotor_chord"] = [9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3

end # module