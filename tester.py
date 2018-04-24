from pool_physics import Ball, Vector, get_shot

CB = Ball(20, 20)
OB = Ball(30, 30)
pocket = Vector(0,0)
hand_1, hand_2 = get_shot(CB, OB, pocket)

print('hand_1:', hand_1, '\nhand_2:', hand_2)