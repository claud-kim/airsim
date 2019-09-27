from drive_controller import DrivingController
import math
import numpy as np
import logging

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False
        self.collision_flag = True
        
        # track type
        self.trackType = 0 # 1:Basic, 2:Racing1,Racing2, 3:Smurf, 4:Marina
        # 구간별 steering 범위
        self.leftAngle = [0.] * 10
        self.rightAngle = [0.] * 10
        self.nA = 0
        
        self.linear_model_coeff = []
        self.linear_model_intercept = 0.
        
        # move
        self.angles = []
        self.angle = 0.
        self.to_middle = 0.
        self.sec = 4.  # 해당 시간동안 속도 v로 달리는 구간

        # 코너별 속도 최적화 
        self.MAX_SPEED = 60.0  # 60x3.6 km
        self.STUCK_SPEED = 5
        
        self.stuckCount = 0
                
        self.half_obs_width = 1.414
        self.carWidth = 2.5
        self.half_car_width = 0.5 * self.carWidth

        self.CAR_LENGTH = 5  # not completed
        self.SMOOTHING_STEER_CONSTANT = 0.05
        self.G_steerFactor = 1.8
        self.last_steering = 0.
        self.backwardTime = 0
        self.bBrake = False
        self.nBrake = 0
        
        self.obstacles_sorted = []
        
        #track 3. 에서 to_middle 값을 steering 에 반영
        self.bMiddle = False
        self.bSpeed = False
        self.bCrossAngle = False
        self.three_count = 0


        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

        self.trackWidth = ( self.half_road_limit - self.half_car_width ) * 2.
        self.half_trackWidth = 0.5 * self.trackWidth

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        angles = sensing_info.track_forward_angles
        angle = sensing_info.moving_angle
        speed = sensing_info.speed
        forward = sensing_info.moving_forward
        way_points = sensing_info.distance_to_way_points
        to_middle = sensing_info.to_middle
        obstacles = sensing_info.track_forward_obstacles

        ###########################################################################


        # ===================================================
        # add function start
        # ===================================================
        # set track type
        def setTrackType():
            if abs(self.half_road_limit - 6.25) < 1.:
                self.trackType = 1 # Map1:Basic Round
            elif angles[0] == -2:
                self.trackType = 2 # Map2:Racing1, 2
            elif angles[1] == -3:
                self.trackType = 3 # Map3:Smurf Village
            elif angles[1] == 0:
                self.trackType = 4 # Map4:Marina
            else:
                pass
        # end function

        # fuction : dist 거리에 있는 구간 찾기
        def find_index(dist):
            ## add code
            if self.trackType == 3 :
                #self.three_count = not self.three_true
                self.three_count += 1
                if self.three_count % 3 < 2 :
                    diffs = []
                    
                    for i in range(1, len(angles)) :
                        diffs.append(angles[i]-angles[i-1])
        
                    if np.linalg.norm(diffs) > np.finfo(float).eps:
                        datas = diffs / (np.linalg.norm(diffs))
                        datas = np.round(np.array(datas)*(np.max(diffs)-np.min(diffs)), 1)
        
                        for i in range(1, len(angles)):
                            angles[i] = angles[i-1] + datas[i-1]
                    else:
                        pass
            

            #end code
        
            return min(len(angles), int(dist / 10)) # idx >= 0
        # end function

        # fuction : [0]~[n] 각도의 최대값이 limit 를 초과할 경우 True
        def isBigCurve(n, limit):
            return False
            if angles[0] > limit:
                return True
            for i in range(n):
                if abs(angles[i]) < 0.01: # 0 angle
                    return False
                elif angles[i]*angles[i+1] < 0.: # 커브방향이 같은 경우
                    return False
                if abs(angles[i+1]) > limit: # 값을 초과하는 경우
                    return True
            return False
        # end function

        # fuction : 전방 도로에 대한 주행 각도(Left/Right) 계산
        def find_pos_angle(idx):
            #dX = self.half_trackWidth - self.half_car_width
            dX = self.half_trackWidth - self.carWidth
            # next 도로 시작점(x, y) : 현재 차량 위치 기준
            x = -to_middle
            y = math.sqrt(way_points[0]**2 - to_middle**2)

            # 결과값 저장
            self.leftAngle[0] = -170.
            self.rightAngle[0] = 170.           
            self.nA = 1
           
            # 현재 구간 
            if idx == 0:
                if y > 1.:
                    self.leftAngle[0] = math.degrees(math.atan((x-dX)/y))
                    self.rightAngle[0] = math.degrees(math.atan((x+dX)/y))
                else:
                    pass
                return y
            
            # startIdx 결정 : 각도변화가 클 경우, [0]~[1] 구간 무시
            self.bSpeed = False
            startIdx = 0
            if self.trackType == 3:
                if angles[1] * angles[2] > 0. and abs(angles[1]) > 13. and abs(angles[2]) > 2 * abs(angles[1]) and angle * angles[0] > 0 and speed > 70.:
                    startIdx = 1
                    self.bSpeed = True
            else:
                if angles[1] * angles[2] > 0. and abs(angles[1]) > 13. and abs(angles[2]) > 2 * abs(angles[1]):
                    startIdx = 1 
                
            # angles[n] 값이 한계값보다 큰 경우, 현재구간 포함하여 각도계산
            thIdx = 2
            thCurve = 70.
            thSum = 0
            
            if isBigCurve(thIdx, thCurve) and y>1.:
                self.leftAngle[self.nA] = (math.degrees(math.atan((x-dX)/y)))
                self.rightAngle[self.nA] = (math.degrees(math.atan((x+dX)/y)))
                self.nA += 1
                for i in range(thIdx+1):
                    thSum += angles[0]
                
            # 다음 구간부터 idx 구간 까지 각도 계산
            for i in range(0, idx):
                theta = math.radians(angles[i])
                x += 10 * math.sin(theta)
                y += 10 * math.cos(theta)
                if i > startIdx: #  현재위치 ~ way_points[1] 무시
                    # backup min, max angle
                    maxLeftAngle = self.leftAngle[self.nA-1]
                    minRightAngle = self.rightAngle[self.nA-1]
                    
                    theta = math.radians(angles[i-1])
                    xL = x - dX * math.cos(theta)
                    xR = x + dX * math.cos(theta)
                    yL = y + dX * math.sin(theta)
                    yR = y - dX * math.sin(theta)
                    
                    # new min, max angle
                    maxLeftAngle = max(maxLeftAngle, math.degrees(math.atan(xL/yL)))
                    minRightAngle = min(minRightAngle, math.degrees(math.atan(xR/yR)))
                    
                    if maxLeftAngle < minRightAngle: # update Left/Right angle
                        self.leftAngle[self.nA] = maxLeftAngle
                        self.rightAngle[self.nA] = minRightAngle
                        self.nA += 1
                    else: # Left, Right angle 이 꼬일 경우, 이전 Left/Right angle return
                        return (i-1) * 10. # 반영 구간
            return idx * 10.
        # end function

        def evaluate_obs_pos(x, y, dist):
            # (xo, yo) : 장애물이 있는 구간의 도로 중앙선상의 점
            resIdx = 0
            if dist <= y: # 현재 구간에 장애물이 있는 경우
                # 장애물까지 거리(dist) 에 해당되는 구간의 끝점의 idx 계산 : way_points[idx]
                c = dist
                xo = x
                yo = dist
                theta = 0.
            else:
                # 장애물까지 거리(dist) 에 해당되는 구간의 끝점의 idx 계산 : way_points[idx]
                resIdx = int((dist - y) / 10.) + 1
                # 끝점에서 장애물까지 거리(c) 계산
                c = dist - (y + 10 * (resIdx - 1))
                
                # (x, y) : 장애물이 있는 구간의 시작점
                for id in range(resIdx-1):
                    theta = math.radians(angles[id])
                    x += 10 * math.sin(theta)
                    y += 10 * math.cos(theta)
                    
                theta = math.radians(angles[resIdx-1])
                xo = x + c * math.sin(theta)
                yo = y + c * math.cos(theta)
            
            return [xo, yo, theta, resIdx]
        # end of evaluate_obs_pos()
                    
        # n개 장애물이 존재하는 거리 까지 장애물 & 도로 폭 고려한 Left/Right 각도 계산..
        def find_obs_pos_angle2(angle, nObstacles, obstacles):
            # way_points[0] 의 x, y 좌표계산 : 현재 차량위치 기준(0,0)
            x = - to_middle
            y = math.sqrt(way_points[0]**2 - to_middle**2)
            
            leftAngle = maxLeftAngle = -170.
            rightAngle = minRightAngle = 170.
            for i in range(nObstacles):
                dist_s = obstacles[i].get('dist')
                obsToMiddle = obstacles[i].get('to_middle')
                [xo, yo, theta, resIdx] = evaluate_obs_pos(x, y, dist_s)
                
                xL = xo + obsToMiddle * math.cos(theta)
                yL = yo - obsToMiddle * math.sin(theta)
                if yL < 0.01:
                    yL = 0.1
                ang = math.degrees(math.atan(xL/yL))
                if obsToMiddle < 0:
                    leftAngle = max(maxLeftAngle, ang)
                else:
                    rightAngle = min(minRightAngle, ang)
    
                if leftAngle < rightAngle:
                    maxLeftAngle = leftAngle
                    minRightAngle = rightAngle
                else:
                    break
            self.nA = nObstacles
            self.leftAngle[self.nA-1] = maxLeftAngle
            self.rightAngle[self.nA-1] = minRightAngle
        # end function

        # n개 장애물이 존재하는 거리 까지 장애물 & 도로 폭 고려한 Left/Right 각도 계산..
        def find_obs_pos_angle(angle, nObstacles, obstacles):
            # way_points[0] 의 x, y 좌표계산 : 현재 차량위치 기준(0,0)
            x = - to_middle
            y = math.sqrt(way_points[0]**2 - to_middle**2)
            idx = 0
             
            b_obstacles = True
            
            # for temporary debugging
            if nObstacles == 10:
                for i in range(nObstacles):
                    obs = obstacles[i]
                    opsDist = obs.get('dist')
                    opsToMiddle = obs.get('to_middle')
                
            i = 0
            self.bCrossAngle = False
            while i < nObstacles and b_obstacles:
                # Obstacle Info
                obs = obstacles[i]
                opsDist = obs.get('dist')
                opsToMiddle = obs.get('to_middle') 
                [xo, yo, theta, idx] = evaluate_obs_pos(x, y, opsDist)
                xL = xo + (opsToMiddle - 1.4 - self.half_car_width) * math.cos(theta)
                xR = xo + (opsToMiddle + 1.4 + self.half_car_width) * math.cos(theta)
                yL = yo - (opsToMiddle - 1.4 - self.half_car_width) * math.sin(theta)
                yR = yo - (opsToMiddle + 1.4 + self.half_car_width) * math.sin(theta)
                
                obsLA = math.degrees(math.atan(xL/yL))
                obsRA = math.degrees(math.atan(xR/yR))
                    
                # 장애물이 leftAngle ~ rightAngle 외부에 있을 때 : 무시함
                if (obsRA <= self.leftAngle[self.nA-1]) or (self.rightAngle[self.nA-1] <= obsLA):
                    i += 1
                    continue
                
                # obsLA <= leftAngle < rightAngle <= obsRA : 장애물이 LA, RA 를 포함할 때
                leftAngle = self.leftAngle[self.nA-1]
                rightAngle = self.rightAngle[self.nA-1]
                if obsLA <= self.leftAngle[self.nA-1] and self.rightAngle[self.nA-1] <= obsRA:
                    #for i in range(self.nA - 1):
                        #id = self.nA - i - 2
                    for j in range(self.nA - 2):
                        id = self.nA - j - 2
                        if obsLA <= self.leftAngle[id] and self.rightAngle[id] <= obsRA:
                            self.leftAngle[self.nA-1] = self.rightAngle[self.nA-1] # Only 장애물
                        else:
                            leftAngle = self.leftAngle[id]
                            rightAngle = self.rightAngle[id]
                            break
                    b_obstacles = False
                    
                # 장애물이 leftAngle 에 걸쳐 있을 때 : leftAngle 조정(obsRA)
                if obsLA <= leftAngle and leftAngle <= obsRA:
                    self.leftAngle[self.nA-1] = obsRA
                    self.rightAngle[self.nA-1] = rightAngle
                # 장애물이 rightAngle 에 걸쳐 있을 때 : rightAngle 조정(obsLA)
                elif obsLA <= rightAngle and rightAngle <= obsRA:
                    self.rightAngle[self.nA-1] = obsLA
                    self.leftAngle[self.nA-1] = leftAngle
                # 장애물 범위가 leftAngle과 rightAngle 사이에 있을 경우 
                else:
                    if to_middle > opsToMiddle:
                        self.leftAngle[self.nA-1] = obsRA
                        self.rightAngle[self.nA-1] = rightAngle
                    else:
                        self.rightAngle[self.nA-1] = obsLA
                        self.leftAngle[self.nA-1] = leftAngle
                
                # Only 장애물: Left/Right 범위로 장애물을 피할 수 없는 경우 최소 dA로 바로앞 장애물만 피해간다
                if self.leftAngle[self.nA-1] >= self.rightAngle[self.nA-1]:
                    if idx > 0:
                        theta = math.radians(angles[idx-1])
                    else:
                        theta = 0.
                    if abs(obsLA - angle) < abs(obsRA - angle):  # (xL, yL) ~ obsLA
                        xL = xo - self.half_trackWidth * math.cos(theta)
                        yL = yo + self.half_trackWidth * math.sin(theta)
                        self.leftAngle[self.nA-1] = math.degrees(math.atan(xL/yL))
                        self.rightAngle[self.nA-1] = obsLA
                    else:  # obsRA ~ (xR, yR)
                        self.leftAngle[self.nA-1] = obsRA
                        xR = xo + self.half_trackWidth * math.cos(theta)
                        yR = yo - self.half_trackWidth * math.sin(theta)
                        self.rightAngle[self.nA-1] = math.degrees(math.atan(xR/yR))

                    b_obstacles = False
                    if self.trackType == 2:
                        self.bCrossAngle = True
                i += 1 # for next obstacles
            # end while loop

        # end function

        # dist 이내에 존재한 장애물 탐지
        def numObstacles(within):
            sub_obstacles = []
            if len(obstacles) == 0:
                return sub_obstacles
            
            nObstacles = 0
            
            self.obstacles_sorted = obstacles
            for obs in self.obstacles_sorted:
                tmpDist = obs.get('dist')
                tM = obs.get('to_middle')
                if tmpDist > 0. and tmpDist <= within and abs(tM) <= self.trackWidth:
                    nObstacles += 1
                    sub_obstacles.append(obs)
                else:
                    return sub_obstacles
            return sub_obstacles
        # end function

        # 충돌 상태 체크
        def isCollision():
            stuckTime = 3
            if sensing_info.collided:
                self.stuckCount += 1 # 충돌 count
                car_controls.steering = -self.last_steering  # 충돌 당시 핸들 반대방향 유지해야 함 (복구 전 자체해결)
                if self.stuckCount > stuckTime and abs(speed) < 6.: # 복구(후진) 시작
                    self.backwardTime = 15
                    if abs(to_middle) < self.half_trackWidth:
                        self.backwardTime = 10                    
                    car_controls.steering = 0.
                    car_controls.throttle = -1.
                    if self.stuckCount > stuckTime + 1: # 후진 시작했으나, 실제 후진이 오랫동안 안될 경우
                        self.stuckCount = 0
                        self.backwardTime = 0
                        car_controls.throttle = 1.0  
                return True # return car_controls
            elif self.backwardTime > 0 : # 충돌 복구 기간(후진 또는 좌/후회전)
                if sensing_info.moving_forward == False:
                    self.backwardTime -= 1
                    car_controls.steering = min(1., max(angle / 68., -1))
                    car_controls.throttle = -1.
                else: # 차량 방향이 반대일 경우
                    self.backwardTime -= 1
                    car_controls.steering = 1. if to_middle > 0. else -1.# 검증해야 함
                    car_controls.throttle = 1.
                return True # return car_controls
            else: # 충돌 reset
                self.stuckCount = 0
                return False
        # end of function

        # steering 각도에 따른 최대 속도 계산
        def find_maxSpeed(myangle):            
            if abs(myangle) > 90 :
                myangle = 89.
            elif myangle == 0 :
                myangle = 0.1 # 169 km/h
            elif myangle <0 :
                myangle = -myangle 
            
            # 루트 (2.25/sin()+1)*u*g*(lr/L) * 3.6 (km/h)
            tt = 2.25/math.sin(math.radians(myangle))+1
            value = math.sqrt(tt*0.7*9.8*(1/4)) * 3.6
            # 1 -> 53.73731880731859
            # 10 -> 17.61303655250063
            # 20 -> 12.978601764809486
            # 30 -> 11.056455128114075
            # 40 -> 10.001360664779224
            # 50 -> 9.354626411307597

            ### 튜닝 요소적인 파라미터
            ## minV :
            ## maxV :
            # 10-30 : o.k
            # 50
            # 1 -> 50
            # 10 -> 20
            minV = 25
            maxV = 40
            if myangle > 0.8 : 
                value = value*3.2 + (minV-maxV)/(50-1)*(myangle-1) + minV 
            if self.trackType == 3:
                return max(value, 40)
            else:
                return max(value, 20)
        # end of function        
           
        # end add function ------------------------------------

        # ============================================================
        # START : control_driving 
        # ============================================================
        # trackType 결정
        if self.trackType == 0:
            setTrackType()

        # 출발 초기 상태
        if sensing_info.lap_progress < 0.6 :
            car_controls.steering = 0
            car_controls.throttle = 1
            return car_controls
            
        dA = 0.            
        if isCollision(): # 충돌 처리)
            return car_controls
        elif sensing_info.moving_forward == False and car_controls.throttle < 0.:
            pass
        else: #정상주행       
            # 최소 30m 전방 고려, sec 초 동안 이동하는 거리 고려
            dist = max(30., self.sec * speed / 3.6)

            # dist 에 해당하는 구간은 끝점 index 계산 (way_points[idx])
            idx = find_index(dist) # idx >= 0
            
            # 목표지점 구간의 끝점에서 경계각도(left, right) 계산
            dist = find_pos_angle(idx) # return 값(idx) : 도로가 꼬이는 경우 return
            
            # dist 내 존재하는 장애물 갯수
            sub_obstacles = numObstacles(dist)
            nObstacles = len(sub_obstacles)
            
            # 장애물 고려한 Left/Right 각도 계산
            self.bMiddle = False
            if nObstacles > 0:                 
                if self.trackType == 3 and nObstacles >= 5:
                    if sub_obstacles[4].get('dist') - sub_obstacles[0].get('dist') < 25.:
                        find_obs_pos_angle2(angle, nObstacles, sub_obstacles)
                        self.bMiddle = True
                    else:
                        find_obs_pos_angle(angle, nObstacles, sub_obstacles)
                else:
                    # n개 장애물이 존재하는 거리 까지 장애물 고려한 Left/Right 각도 계산..
                    find_obs_pos_angle(angle, nObstacles, sub_obstacles)
                
            # steering 값 결정 : abs(dA) 를 최소화 하는 값으로 결정
            if angle < self.leftAngle[self.nA-1]:
                dA = self.leftAngle[self.nA-1] - angle
            elif self.rightAngle[self.nA-1] < angle:
                dA = self.rightAngle[self.nA-1] - angle
            
            #------------------------------------
            # 회전각(dA) 에 의한 sttering 값 결정 
            # -----------------------------------
            
            if self.bMiddle: # track 3 to_middle 반영
                car_controls.steering = dA/68. - to_middle / self.trackWidth
            else: # Moving straight forward
                scale_param = 1.
                if nObstacles > 0: # 장애물이 있는 경우 steering 값을 증가시켜준다
                    scale_param = 1/(nObstacles + 1)
                car_controls.steering = min(1., max(dA / 68./ scale_param, -1))

            # throttle, brake 조정.
            car_controls.throttle = 1.
            car_controls.brake = 0.
            
            forwardAngle = car_controls.steering*50
            speedMax = find_maxSpeed(forwardAngle)
            forwardSpeed = speed*math.cos(math.radians(forwardAngle))
            
            # 장애물이 없는 도로에서 커브길을 고려하여 brake 값 조절
            sumAng = sum(angles)
            if nObstacles == 0:           
                if self.bBrake == True :
                    if self.nBrake > 0: # brake 진행중
                        car_controls.brake = 1.
                        car_controls.throttle = 0.
                        self.nBrake -= 1
                    else: # brake 해제
                        self.bBrake = False
                        self.nBrake = 0
                else : # braking condition
                    if speed > 130. and angles[1] == 0. and abs(angles[7]) > 5.:
                        car_controls.brake = 1.
                        car_controls.throttle = 0.
                        self.bBrake = True
                        self.nBrake = round((speed-110.)/3.)
                    elif speed > 92 and abs(sumAng)>900 :
                        car_controls.brake = 1.
                        car_controls.throttle = 0.
                        self.bBrake = True
                        self.nBrake = 3
                        
            # 장애물과 관계없이 속도에 따른 회전각 한계를 초과하는 경우 steering 조정  
            if (forwardSpeed > speedMax and abs(forwardAngle)>1.) or self.bSpeed:
                car_controls.brake = 1.
                car_controls.throttle = 0.
                if self.trackType == 3 or self.trackType == 4:
                    car_controls.steering = car_controls.steering*0.8
                else:
                    car_controls.steering = car_controls.steering*0.5
                
            if self.trackType == 2 and 1.5 < sensing_info.lap_progress < 4.:
                car_controls.throttle = 0.7
            
            # end : 커브길 이전 속도조절
            
        # End 정상 주행  ===============================================
        self.last_steering = car_controls.steering
        
        # for debug
        if abs(angles[0]) < 0.5:
            strD = "직"
        elif angles[0] < 0.:
            strD = "좌"
        else:
            strD = "우"

        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    client = DrivingClient()
    client.run()
