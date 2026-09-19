import pygame, time, datetime
pygame.init(); info=pygame.display.Info(); W,H=info.current_w,info.current_h
s=pygame.display.set_mode((W,H),pygame.FULLSCREEN); print("size",W,H,"driver",pygame.display.get_driver(),flush=True)
big=pygame.font.SysFont(None,int(H*0.25)); small=pygame.font.SysFont(None,int(H*0.09))
cols=[(255,0,0),(0,255,0),(0,0,255),(255,255,255)]; t0=time.time()
while time.time()-t0<20:
    i=int((time.time()-t0)//1)%4; s.fill((20,20,20)); pygame.draw.rect(s,cols[i],(0,0,W,int(H*0.18)))
    s.blit(big.render("MONITOR OK",True,(255,255,0)),(int(W*0.06),int(H*0.3)))
    s.blit(small.render(f"{W}x{H}  {datetime.datetime.now():%H:%M:%S}",True,(200,200,200)),(int(W*0.06),int(H*0.7)))
    pygame.display.flip(); pygame.event.pump(); time.sleep(0.2)
pygame.quit()
