const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(process.argv[2] || __dirname + '/../dcaron_bridge/web_panel.html', 'utf8').split('<script>')[1].split('</script>')[0];
const windowEvents = {}, docEvents = {}, intervals = [], calls = [];
let now = 0;
const elements = {};
for (const id of ['status','speed','position','yaw','vx','vy','wz','age','stop','linear','linear-range','angular','angular-range','apply','notice','control-hz','odom-hz','heartbeat','subscribers','clear-trail','grid-lines','axis-labels','trail','car','scale','plot','gesture','gesture-line','gesture-target','view-mode','drive-mode','zoom-in','zoom-out','reset-view','canvas-hint']) elements[id] = {textContent:'',value:'',events:{},style:{},classList:{toggle(){}},addEventListener(name,fn){this.events[name]=fn;},setPointerCapture(){},getBoundingClientRect(){return {left:0,top:0,width:900,height:600};},setAttribute(name,value){this[name]=value;}};
const buttons = 'wsadqe'.split('').map(key => ({dataset:{key}, classList:{toggle(){}},
    events:{}, addEventListener(name, fn){this.events[name]=fn;}, setPointerCapture(){}}));
const document = {hidden:false, hasFocus:()=>true,
    getElementById:id=>elements[id],
    querySelector:id=>elements[id.slice(1)], querySelectorAll:()=>buttons,
    addEventListener:(event, fn)=>docEvents[event]=fn};
const context = vm.createContext({document, window:{addEventListener:(event,fn)=>windowEvents[event]=fn},
    performance:{now:()=>now}, AbortSignal, Date, Math,
    setInterval:fn=>intervals.push(fn), setTimeout:()=>{},
    fetch:async(path, options)=>{
        if(path==='/command') calls.push(JSON.parse(options.body));
        if(path==='/speed') calls.push(JSON.parse(options.body));
        return {ok:true,json:async()=>({age:0.1,subscribers:1,linear:0.15,angular:0.4,control_hz:50,control_target_hz:50,odom_hz:10,heartbeat_target_hz:16.7,odom:{x:1,y:2,yaw:90,vx:0,vy:0,wz:0}})};
    }});
const flush = ()=>new Promise(resolve=>setImmediate(resolve));
const event = (key,repeat=false)=>({key,repeat,target:{tagName:'BODY'},preventDefault(){}});
(async()=>{
    vm.runInContext(source,context);
    await flush();
    assert.equal(elements['control-hz'].textContent,'50.0 / 50 Hz');
    assert.equal(elements['odom-hz'].textContent,'10.0 Hz');
    assert(elements.car.transform.includes('rotate(-90)'));
    windowEvents.keydown(event('w'));
    assert.equal(calls.at(-1).key,'w');
    now=60; intervals[0]();
    assert.equal(calls.at(-1).key,'w'); // hold works before first system repeat
    windowEvents.keydown(event('q'));
    assert.equal(calls.at(-1).key,'qw');
    windowEvents.keyup(event('q'));
    assert.equal(calls.at(-1).key,'w'); // releasing turn preserves held forward
    windowEvents.keyup(event('w'));
    assert.equal(calls.at(-1).key,'');
    const length=calls.length; now=120; intervals[0]();
    assert.equal(calls.length,length); // no motion heartbeats after release
    windowEvents.keydown(event('w',true));
    assert.equal(calls.length,length); // ignored stale key repeat
    windowEvents.keydown(event('a')); windowEvents.blur();
    assert.equal(calls.at(-1).key,'');
    windowEvents.keydown(event('q')); now=1000; intervals[0]();
    assert.equal(calls.at(-1).key,''); // long browser pause cannot retain motion
    elements.linear.value='0.25'; elements.angular.value='0.6';
    await elements.apply.onclick();
    assert.equal(calls.at(-1).linear,0.25);
    assert.equal(calls.at(-1).angular,0.6);
    elements.linear.value='0.01'; elements.angular.value='0.01';
    await elements.apply.onclick();
    assert.equal(calls.at(-1).linear,0.01);
    assert.equal(calls.at(-1).angular,0.01);
    const before=calls.length;
    windowEvents.keydown({...event('w'),target:{tagName:'INPUT'}});
    assert.equal(calls.length,before); // editing speed never starts movement
    const plot=elements.plot;
    // Browsing must move the camera without sending robot commands.
    const browseBefore=calls.length;
    plot.events.pointerdown({button:0,pointerId:8,clientX:400,clientY:300,preventDefault(){}});
    plot.events.pointermove({pointerId:8,clientX:450,clientY:330,preventDefault(){}});
    assert.equal(calls.length,browseBefore);
    assert(vm.runInContext('camera !== null',context));
    plot.events.pointerup({pointerId:8});
    const worldBefore=vm.runInContext('[(550-plotView.cx)/plotView.unit,(plotView.cy-320)/plotView.unit]',context);
    const unitBefore=vm.runInContext('plotView.unit',context);
    plot.events.wheel({clientX:550,clientY:320,deltaY:-100,preventDefault(){}});
    const worldAfter=vm.runInContext('[(550-plotView.cx)/plotView.unit,(plotView.cy-320)/plotView.unit]',context);
    assert(Math.abs(worldBefore[0]-worldAfter[0])<1e-9);
    assert(Math.abs(worldBefore[1]-worldAfter[1])<1e-9);
    assert(vm.runInContext('plotView.unit',context)>unitBefore);
    assert.equal(calls.length,browseBefore);
    plot.events.pointerdown({button:0,pointerType:'touch',pointerId:11,clientX:300,clientY:300,preventDefault(){}});
    plot.events.pointerdown({button:0,pointerType:'touch',pointerId:12,clientX:500,clientY:300,preventDefault(){}});
    const pinchUnit=vm.runInContext('plotView.unit',context);
    plot.events.pointermove({pointerId:12,clientX:600,clientY:300,preventDefault(){}});
    assert(vm.runInContext('plotView.unit',context)>pinchUnit);
    plot.events.pointerup({pointerId:11}); plot.events.pointerup({pointerId:12});
    assert.equal(calls.length,browseBefore);
    elements['reset-view'].onclick();
    assert(vm.runInContext('camera === null',context));
    const finalBefore=calls.length;
    plot.events.pointerdown({button:0,pointerId:9,clientX:500,clientY:400,preventDefault(){}});
    plot.events.pointermove({pointerId:9,clientX:560,clientY:430,preventDefault(){}});
    plot.events.pointerup({pointerId:9});
    assert.equal(calls.length,finalBefore);
    assert(!source.includes('targetDirection'));
    assert(!source.includes('drive-mode'));
    console.log('浏览器事件测试通过：按住、松键、重复、失焦、鼠标释放、计时器暂停');
})().catch(e=>{console.error(e);process.exitCode=1;});
