"""Native synchronized station map and perspective bubble viewer."""
from __future__ import annotations
import math
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk
import numpy as np
from PIL import Image, ImageTk
from database_backend import open_database
from panorama import ensure_panorama, perspective
from station_model import project_hotspot, suggest_neighbors


class ScanBrowser(tk.Toplevel):
    def __init__(self,parent,path,scratch,memory,cache_dir=None):
        super().__init__(parent)
        self.title('PointCloud Filter 3.1 | Scan Map + Bubble Viewer')
        self.geometry('1200x760'); self.configure(bg='#18212c')
        self.path,self.scratch,self.memory=path,scratch,memory
        self.stations=[]; self.edges=[]; self.current=None
        self.panorama=None; self.yaw=0.; self.pitch=0.; self.fov=85
        self.events=queue.Queue(); self.stop=threading.Event(); self.worker=None
        self.cache=Path(cache_dir) if cache_dir else path.with_name(path.stem+'_panoramas')
        self.scale=1.; self.offset=np.array([200.,200.]); self.drag=None
        self.hotspots=[]; self.closed=False
        bar=tk.Frame(self,bg='#18212c');bar.pack(fill='x',padx=10,pady=8)
        self.status=tk.Label(bar,text='Loading stations...',fg='white',bg='#18212c',anchor='w')
        self.status.pack(side='left',fill='x',expand=True)
        self.button(bar,'Cancel load',self.cancel).pack(side='right')
        self.button(bar,'Fullscreen',self.toggle_fullscreen).pack(side='right',padx=4)
        body=tk.PanedWindow(self,orient='horizontal',bg='#18212c',sashwidth=8);body.pack(fill='both',expand=True,padx=10)
        left=tk.Frame(body,bg='#18212c');body.add(left,width=380)
        self.map=tk.Canvas(left,bg='#0f1722',highlightthickness=0,height=320)
        self.map.pack(fill='both',expand=True)
        self.map.bind('<ButtonPress-1>',self.map_press)
        self.map.bind('<B1-Motion>',self.map_drag)
        self.map.bind('<ButtonRelease-1>',self.map_release)
        self.map.bind('<MouseWheel>',self.map_zoom)
        self.map.bind('<Configure>',lambda e:self.draw_map())
        tk.Label(left,text='Local/project XY (meters). Drag to pan; wheel to zoom.\nMissing poses remain in the list.',fg='#b5c9db',bg='#18212c').pack()
        self.list=tk.Listbox(left,bg='#0f1722',fg='white',selectbackground='#237a98',exportselection=False,height=8)
        self.list.pack(fill='x',pady=8);self.list.bind('<<ListboxSelect>>',self.list_select)
        right=tk.Frame(body,bg='#18212c');body.add(right)
        self.bubble=tk.Canvas(right,bg='black',highlightthickness=0)
        self.bubble.pack(fill='both',expand=True)
        self.bubble.bind('<Configure>',lambda e:self.render())
        self.bubble.bind('<ButtonPress-1>',self.bubble_press)
        self.bubble.bind('<B1-Motion>',self.bubble_drag)
        self.bubble.bind('<MouseWheel>',self.bubble_zoom)
        nav=tk.Frame(right,bg='#18212c');nav.pack(fill='x',pady=6)
        self.previous=ttk.Combobox(nav,state='readonly',width=23);self.previous.pack(side='left')
        self.previous.bind('<<ComboboxSelected>>',lambda e:self.navigate(False))
        self.next=ttk.Combobox(nav,state='readonly',width=23);self.next.pack(side='left',padx=5)
        self.next.bind('<<ComboboxSelected>>',lambda e:self.navigate(True))
        tk.Label(right,text='Drag to look around; wheel to zoom. Click station markers to move.',fg='#b5c9db',bg='#18212c').pack()
        edits=tk.Frame(self,bg='#18212c');edits.pack(fill='x',padx=10,pady=8)
        tk.Label(edits,text='Selected station → downstream:',fg='white',bg='#18212c').pack(side='left')
        self.target=ttk.Combobox(edits,state='readonly',width=42);self.target.pack(side='left',padx=6)
        self.button(edits,'Add connection',lambda:self.edit_edge(True)).pack(side='left')
        self.button(edits,'Remove connection',lambda:self.edit_edge(False)).pack(side='left',padx=5)
        self.button(edits,'Suggest nearest',self.suggest).pack(side='left')
        self.info=tk.Label(self,text=f'Cache: {self.cache}',fg='#b5c9db',bg='#18212c',anchor='w',wraplength=1150,justify='left')
        self.info.pack(fill='x',padx=10,pady=(0,8))
        self.protocol('WM_DELETE_WINDOW',self.close_browser)
        self.bind('<Escape>',lambda e:self.attributes('-fullscreen',False))
        self.after(100,self.poll_events)
        self.start_task(self.load_project)

    @staticmethod
    def button(parent,text,command):
        return tk.Button(parent,text=text,command=command,bg='#294861',fg='white',relief='flat',padx=8,pady=5)

    def toggle_fullscreen(self):
        self.attributes('-fullscreen',not self.attributes('-fullscreen'))

    def start_task(self,task):
        if self.worker and self.worker.is_alive():return False
        self.stop.clear()
        def run():
            try:task()
            except InterruptedError:self.events.put(('status','Cancelled'))
            except Exception as exc:self.events.put(('status',f'Error: {exc}'))
        self.worker=threading.Thread(target=run,daemon=True);self.worker.start()
        return True

    def database(self):
        return open_database('DuckDB',self.path,self.scratch,self.memory)

    def load_project(self):
        with self.database() as db:
            stations=db.list_stations()
            # Upgrade metadata only: original-resolution point rows stay intact.
            for s in stations:
                if self.stop.is_set():raise InterruptedError
                if s['index']<0 or s['meta'].get('version'):continue
                try:
                    import pye57
                    from pointcloud_filter import Options,ScanJob,scan_source_identity
                    from station_model import inspect_station
                    source=Path(s['path'])
                    opt=Options(source.parent,source.parent,self.scratch,apply_e57_pose=s['posed'])
                    if scan_source_identity(ScanJob(source,s['index']),opt)[1]!=s['signature']:
                        continue
                    with pye57.E57(str(source)) as e:s['meta']=inspect_station(e,s['index'])
                    db.save_station(s['id'],s['meta'])
                except Exception as exc:
                    s['meta']['metadata_error']=str(exc)
            self.events.put(('project',(stations,db.list_connections())))

    def poll_events(self):
        try:
            while True:
                kind,data=self.events.get_nowait()
                if kind=='status':self.status.configure(text=data)
                elif kind=='project':
                    self.stations,self.edges=data
                    self.list.delete(0,'end')
                    for s in self.stations:self.list.insert('end',self.label(s))
                    self.target['values']=[self.label(s) for s in self.stations]
                    positioned=[s['meta']['position'][:2] for s in self.stations if s['meta'].get('position') is not None]
                    if positioned:
                        points=np.asarray(positioned);mid=(points.min(0)+points.max(0))/2
                        self.scale=280/max(float(np.ptp(points,axis=0).max()),1.)
                        self.offset=np.array([180.,160.])-mid*np.array([self.scale,-self.scale])
                    self.draw_map();self.status.configure(text=f'{len(self.stations)} current completed stations')
                elif kind=='panorama':
                    key,array,meta=data
                    if key==self.current['key']:
                        self.current['meta']=meta;self.panorama=array
                        self.status.configure(text=self.label(self.current))
                        report=f'{meta["panorama_kind"]}; associated images: {len(meta.get("images",[]))}. Cache: {self.cache}'
                        if meta.get('image_error'):report+='; image fallback: '+meta['image_error']
                        self.info.configure(text=report);self.render()
                elif kind=='edges':
                    self.edges=data;self.draw_map();self.update_navigation();self.render()
        except queue.Empty:pass
        if not self.closed:self.after(100,self.poll_events)

    @staticmethod
    def label(s):
        return f'{s["id"]}: {s["meta"].get("name",s["label"])}'

    def select_station(self,key):
        if self.worker and self.worker.is_alive():
            self.status.configure(text='Loading: wait or cancel before selecting another station.');return
        self.current=next(s for s in self.stations if s['key']==key)
        self.panorama=None;self.yaw=0.;self.pitch=0.
        self.list.selection_clear(0,'end');self.list.selection_set(self.stations.index(self.current))
        self.list.see(self.stations.index(self.current))
        self.draw_map();self.update_navigation();self.render()
        station=self.current
        def load():
            with self.database() as db:
                path,meta=ensure_panorama(db,station,self.cache,self.stop,lambda msg:self.events.put(('status',msg)))
                with Image.open(path) as image:array=np.array(image.convert('RGB'))
            self.events.put(('panorama',(key,array,meta)))
        self.status.configure(text='Loading panorama...');self.start_task(load)

    def list_select(self,event):
        selected=self.list.curselection()
        if selected:self.select_station(self.stations[selected[0]]['key'])

    def update_navigation(self):
        if not self.current:return
        key=self.current['key']
        self.prev_keys=[a for a,b in self.edges if b==key]
        self.next_keys=[b for a,b in self.edges if a==key]
        bykey={s['key']:s for s in self.stations}
        self.previous['values']=[self.label(bykey[k]) for k in self.prev_keys]
        self.next['values']=[self.label(bykey[k]) for k in self.next_keys]
        self.previous.set('Previous / upstream');self.next.set('Next / downstream')

    def navigate(self,forward):
        combo=self.next if forward else self.previous
        keys=self.next_keys if forward else self.prev_keys
        i=combo.current()
        if i>=0:self.select_station(keys[i])

    def edit_edge(self,connected):
        if not self.current or self.target.current()<0:return
        source=self.current['key'];target=self.stations[self.target.current()]['key']
        def save():
            with self.database() as db:
                db.set_connection(source,target,connected)
                self.events.put(('edges',db.list_connections()))
        self.start_task(save)

    def suggest(self):
        if not self.current:return
        keys=suggest_neighbors(self.stations,self.current['key'])
        names=[self.label(s) for k in keys for s in self.stations if s['key']==k]
        self.info.configure(text='Nearest suggestions (not a flow direction): '+', '.join(names)+'. Choose a target and Add connection to define downstream.')
        if keys:self.target.current(next(i for i,s in enumerate(self.stations) if s['key']==keys[0]))

    def xy(self,s):
        p=s['meta'].get('position')
        return np.asarray(p[:2])*np.array([self.scale,-self.scale])+self.offset if p is not None else None

    def draw_map(self):
        self.map.delete('all');bykey={s['key']:s for s in self.stations}
        for a,b in self.edges:
            p,q=self.xy(bykey[a]),self.xy(bykey[b])
            if p is not None and q is not None:self.map.create_line(*p,*q,fill='#49657d',arrow='last')
        for s in self.stations:
            p=self.xy(s)
            if p is None:continue
            x,y=p;color='#ffce6b' if self.current and s['key']==self.current['key'] else '#58cad3'
            self.map.create_oval(x-6,y-6,x+6,y+6,fill=color,outline='')
            self.map.create_text(x+10,y-10,text=self.label(s),fill=color,anchor='w')

    def map_press(self,e):self.drag=(e.x,e.y);self.map_start=(e.x,e.y);self.map_moved=False
    def map_drag(self,e):
        delta=np.array([e.x-self.drag[0],e.y-self.drag[1]])
        self.offset+=delta;self.drag=(e.x,e.y);self.map_moved=True;self.draw_map()
    def map_release(self,e):
        if self.map_moved:return
        for s in self.stations:
            p=self.xy(s)
            if p is not None and np.linalg.norm(p-[e.x,e.y])<12:self.select_station(s['key']);break
    def map_zoom(self,e):
        factor=1.15 if e.delta>0 else 1/1.15
        anchor=np.array([e.x,e.y]);self.offset=anchor+(self.offset-anchor)*factor
        self.scale*=factor;self.draw_map()

    def bubble_press(self,e):
        self.drag=None
        for x,y,key in self.hotspots:
            if math.hypot(e.x-x,e.y-y)<18:self.select_station(key);return
        self.drag=(e.x,e.y)
    def bubble_drag(self,e):
        if self.drag is None:return
        self.yaw+=(e.x-self.drag[0])*.005
        self.pitch=np.clip(self.pitch+(e.y-self.drag[1])*.005,-1.5,1.5)
        self.drag=(e.x,e.y);self.render()
    def bubble_zoom(self,e):
        self.fov=float(np.clip(self.fov-(5 if e.delta>0 else -5),30,115));self.render()

    def render(self):
        self.bubble.delete('all');self.hotspots=[]
        if self.panorama is None:
            self.bubble.create_text(180,80,text='Select a station to load its bubble view',fill='white');return
        w=max(1,self.bubble.winfo_width());h=max(1,self.bubble.winfo_height())
        # Bound CPU projection work even in fullscreen; scale final bitmap.
        ratio=min(1.,800/w,600/h);rw=max(1,int(w*ratio));rh=max(1,int(h*ratio))
        image=perspective(self.panorama,self.yaw,self.pitch,self.fov,rw,rh).resize((w,h))
        self.photo=ImageTk.PhotoImage(image);self.bubble.create_image(0,0,image=self.photo,anchor='nw')
        meta=self.current['meta'];origin=meta.get('position')
        if origin is None:return
        rotation=meta.get('view_rotation') or meta.get('rotation')
        key=self.current['key'];neighbors={b if a==key else a for a,b in self.edges if key in (a,b)}
        for target in self.stations:
            if target['key'] not in neighbors or target['meta'].get('position') is None:continue
            vector=(np.asarray(target['meta']['position'])-origin)@np.asarray(rotation)
            point=project_hotspot(vector,self.yaw,self.pitch,self.fov,w,h)
            if point:
                x,y=point;self.hotspots.append((x,y,target['key']))
                self.bubble.create_oval(x-12,y-12,x+12,y+12,fill='#198da4',outline='white',width=2)
                self.bubble.create_text(x,y+25,text=self.label(target),fill='white')

    def cancel(self):self.stop.set()
    def close_browser(self):
        self.stop.set()
        if self.worker and self.worker.is_alive():
            self.status.configure(text='Cancelling; waiting for current batch...')
            self.after(100,self.close_browser);return
        self.closed=True;self.destroy()
