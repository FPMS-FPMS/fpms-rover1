#!/usr/bin/env python3
"""Load each tab in its OWN fresh page and report whether the module mounted.
A hash change does not reload the page, so tabs must be checked separately or
a panel caught mid-mount reads as broken."""
import base64,json,os,socket,struct,subprocess,sys,time,urllib.request
BASE=9400
def ws_send(s,p):
    h=bytearray([0x81]); n=len(p); m=os.urandom(4)
    if n<126: h.append(0x80|n)
    elif n<65536: h.append(0x80|126); h+=struct.pack(">H",n)
    else: h.append(0x80|127); h+=struct.pack(">Q",n)
    h+=m; s.sendall(bytes(h)+bytes(b^m[i%4] for i,b in enumerate(p)))
def rx(s,n):
    b=b""
    while len(b)<n:
        c=s.recv(n-len(b))
        if not c: raise ConnectionError
        b+=c
    return b
def ws_recv(s):
    b0,b1=rx(s,2); op=b0&0xF; ln=b1&0x7F
    if ln==126: ln=struct.unpack(">H",rx(s,2))[0]
    elif ln==127: ln=struct.unpack(">Q",rx(s,8))[0]
    if b1&0x80: rx(s,4)
    return op,rx(s,ln)
def ws_open(u):
    _,rest=u.split("://",1); hp,path=rest.split("/",1); h,p=hp.split(":")
    s=socket.create_connection((h,int(p)),timeout=25)
    k=base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET /{path} HTTP/1.1\r\nHost: {hp}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {k}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
    r=b""
    while b"\r\n\r\n" not in r: r+=s.recv(4096)
    return s
def check(tab, port):
    prof=os.environ["CLAUDE_JOB_DIR"]+f"/tmp/chrome-tc-{tab}"
    os.system(f"rm -rf {prof}")
    ch=subprocess.Popen(["google-chrome","--headless=new","--disable-gpu","--no-sandbox",
        "--no-first-run","--disable-dev-shm-usage",f"--remote-debugging-port={port}",
        f"--user-data-dir={prof}",f"http://127.0.0.1:8770/#{tab}"],
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        tgt=None
        for _ in range(70):
            try:
                for t in json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list",timeout=3)):
                    if t.get("type")=="page" and "8770" in t.get("url",""): tgt=t["webSocketDebuggerUrl"];break
                if tgt: break
            except Exception: pass
            time.sleep(0.5)
        if not tgt: return "no target"
        s=ws_open(tgt); mid=[0]; errs=[]
        ws_send(s,json.dumps({"id":900,"method":"Runtime.enable"}).encode())
        ws_send(s,json.dumps({"id":901,"method":"Log.enable"}).encode())
        time.sleep(16)
        mid[0]+=1; i=mid[0]
        ws_send(s,json.dumps({"id":i,"method":"Runtime.evaluate","params":{
            "expression":"(()=>{const st=document.getElementById('stage');"
                         "const t=st?st.innerText:'';"
                         "return JSON.stringify({err:/COULD NOT LOAD/i.test(t),"
                         "head:t.slice(0,110).replace(/\\s+/g,' ')});})()",
            "returnByValue":True}}).encode())
        s.settimeout(2); end=time.time()+12; got=None
        while time.time()<end:
            try: op,d=ws_recv(s)
            except Exception: continue
            if op!=0x1: continue
            m=json.loads(d)
            if m.get("method")=="Runtime.exceptionThrown":
                dd=m["params"]["exceptionDetails"]
                errs.append(str((dd.get("exception") or {}).get("description",""))[:100])
            if m.get("id")==i:
                got=((m.get("result") or {}).get("result") or {}).get("value"); break
        if not got: return "no result"
        d=json.loads(got)
        if d["err"]: return "FAILED TO LOAD -> " + d["head"]
        return "ok  | " + d["head"]
    finally:
        ch.terminate()
        try: ch.wait(timeout=6)
        except Exception: ch.kill()
for n,tab in enumerate(["control","lidar","cameras"]):
    print(f"  {tab:9s} {check(tab, BASE+n)}")
