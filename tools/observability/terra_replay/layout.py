import rerun.blueprint as rrb


def default_blueprint(roots_and_frames, unpaired_roots=()):
    """Explicit paths and pinhole origins keep each calibrated image in its own frame."""
    world=[];camera=[];depth=[];occupancy=[];metrics=[]
    def label(base,number):return base if number==0 else f'{base} {number+1}'
    for root, frames in sorted(roots_and_frames.items()):
        world += [root+'/route',root+'/pose']
        metrics.append(root+'/metrics/**')
        for frame in frames:
            geometry=root+'/frames/'+frame
            world += [geometry+'/classified_endpoints',geometry+'/camera/**',geometry+'/ground',geometry+'/obstacle_min',geometry+'/obstacle_max',geometry+'/occupancy']
            camera.append(rrb.Spatial2DView(name=label('Camera',len(camera)),origin=geometry+'/camera/rgb',contents=[geometry+'/camera/rgb/image']))
            depth.append(rrb.Spatial2DView(name=label('Depth',len(depth)),origin=geometry+'/camera/depth',contents=[geometry+'/camera/depth/image']))
            occupancy.append(rrb.Spatial2DView(name=label('Occupancy',len(occupancy)),origin=geometry+'/occupancy_image',contents=[geometry+'/occupancy_image']))
        if root in unpaired_roots:
            camera.append(rrb.Spatial2DView(name='Unpaired camera',origin=root+'/receipt_only',contents=[root+'/receipt_only/rgb']))
    def tabs(views,title):
        if not views:return rrb.Spatial2DView(name=title+' unavailable',origin='/no_evidence',contents=[])
        if len(views)==1:return views[0]
        return rrb.Tabs(*views,active_tab=len(views)-1,name=title)
    return rrb.Blueprint(rrb.Vertical(
        rrb.Horizontal(
            rrb.Spatial3DView(name='World, ground and planning',origin='/rovers',contents=world),
            tabs(camera,'Camera'),tabs(depth,'Depth'),tabs(occupancy,'Occupancy'),
        ),
        rrb.TimeSeriesView(name='Motion, freshness and connections',contents=metrics),
        rrb.TextLogView(name='Operational events',origin='/'),
        row_shares=[0.55,0.2,0.25],
    ),rrb.TimePanel(timeline='wall_clock_approx'),collapse_panels=False)
