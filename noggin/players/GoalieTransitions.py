from ..playbook import PBConstants
from .. import NogginConstants as Constants
#arbitrary, currently same as AIBO
BALL_SAVE_LIMIT_TIME = 2.5
MOVE_TO_SAVE_DIST_THRESH = 200.

def shouldSave(player):
    ball = player.brain.ball

    if ball.on:
        relX = ball.relX
    else:
        relX = ball.locRelX

    # Test velocity values as to which one would work:
    relVelX = ball.relVelX
    relVelY = ball.relVelY
    if relVelX < 0.0:
        timeUntilSave = relX / -relVelX
    else:
        timeUntilSave = -1
        '''player.printf(("relVelX = ", relVelX, " relVelY = ", relVelY,
        " timeUntilSave = ", timeUntilSave,
        " anticipatedY = ", anticipatedY))'''
    # No Time, Save now
    if (timeUntilSave < BALL_SAVE_LIMIT_TIME and
        ball.framesOn > 3. and relVelX < 0.
        and 0 < relX < MOVE_TO_SAVE_DIST_THRESH):
        #player.brain.sensors.saveFrame()
        #player.printf("relVelX = %g  relVelY = %g  timeUntilSave = %g" %
                      #(relVelX, relVelY, timeUntilSave))
        return True;
    return False;

def shouldHoldSave(player):
    # same as shouldSave() except for the ball.framesOn check
    # try to come up with better conditions to test
    '''if the ball is still in front of me and coming at me, hold save
    if it's going to arrive anytime soon'''
    ball = player.brain.ball

    if ball.on:
        relX = ball.relX
    else:
        relX = ball.locRelX
    relVelX = ball.relVelX
    if relVelX < 0.0:
        timeUntilSave = relX/ -relVelX
    else:
        timeUntilSave = -1
    if 0 < timeUntilSave < BALL_SAVE_LIMIT_TIME*2 and relVelX < 0 and\
            0 < relX < MOVE_TO_SAVE_DIST_THRESH:
        return True
    return False

def shouldChase(player):
    return False
    if (player.brain.playbook.subRole == PBConstants.GOALIE_CHASER)\
        and (0 < player.brain.ball.locDist <= 60):
        return True
    return False

def shouldStopChaseLoc(player):
    my = player.brain.my
    if (my.x > Constants.MY_GOALBOX_RIGHT_X + PBConstants.END_CLEAR_BUFFER
        or my.y > Constants.MY_GOALBOX_TOP_Y + PBConstants.END_CLEAR_BUFFER
        or my.y < Constants.MY_GOALBOX_BOTTOM_Y + PBConstants.END_CLEAR_BUFFER):
        return True
    return False

def shouldStopChase(player):
    if player.brain.ball.locDist > 60:
        return True
    return False


def ballBtwnMeAndGoal(player):
    '''circle the ball if it's between us and our own goal'''
    brain = player.brain
    if brain.ball.x < brain.my.x:
        return True
    if ((90 <= brain.my.h <= 180) or (-180 <= brain.my.h <= -90))\
        and brain.ball.on:
        #and my.uncertH < ?
        return True