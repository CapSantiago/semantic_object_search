'use strict';

(() => {
  const el = (id) => document.getElementById(id);
  const labels = {
    starting: '启动中', booting: '启动中', preflight: '启动检查', checking: '启动检查', simulation: '启动仿真',
    waiting_simulation: '等待传感器', navigation: '启动导航', waiting_navigation: '等待定位导航',
    perception: '启动感知', waiting_perception: '等待视觉数据', ready: '系统就绪',
    running: '执行中', canceling: '正在停止', completed: '已完成', succeeded: '已完成',
    found: '已找到', passed: '检查通过', partial: '部分完成', failed: '失败', error: '异常',
    canceled: '已取消', stopped: '已停止', stopping: '正在关闭', shutdown: '已关闭',
    not_found: '未找到', no_sensor_data: '缺少感知数据', localization_failed: '定位失败',
    navigation_failed: '导航失败', navigation_rejected: '导航未就绪', timeout: '执行超时',
    idle: '空闲', pending: '等待启动', exited: '已退出', disabled: '未启用',
  };
  const componentLabels = {simulation: 'Gazebo', navigation: '导航', detector: '视觉检测', search: '搜索服务', rviz: 'RViz'};
  const categories = {cup: '杯子', bottle: '瓶子', chair: '椅子'};
  const rooms = {};
  let roomSignature = '';
  const busyStates = new Set(['starting', 'running', 'canceling', 'queued', 'pending']);
  let state = null;
  let online = false;
  let mutationPending = false;
  let selectedHistoryId = null;
  let lastNotice = '';
  let mapData = null;
  let mapRaster = null;
  let mapUnavailable = false;
  let zoom = 1;
  let historyKey = '';
  let taskKey = '';
  let logBusy = false;
  const canvas = el('map');
  const ctx = canvas.getContext('2d');

  const text = (id, value) => { el(id).textContent = value; };
  const finite = (n) => typeof n === 'number' && Number.isFinite(n);
  const statusLabel = (value) => labels[value] || value || '等待数据';
  const statusClass = (value) => {
    if (['ready', 'found', 'completed', 'succeeded', 'passed'].includes(value)) return 'good';
    if (['failed', 'error', 'no_sensor_data', 'localization_failed', 'navigation_failed', 'timeout'].includes(value)) return 'bad';
    if (['partial', 'canceling', 'starting', 'booting', 'running'].includes(value)) return 'warn';
    return 'neutral';
  };
  function badge(id, value, label) {
    const node = el(id);
    node.className = 'badge ' + statusClass(value);
    node.textContent = label || statusLabel(value);
  }
  function announce(message, error = false) {
    lastNotice = message;
    const node = el('notice');
    node.textContent = message;
    node.className = 'notice' + (error ? ' error' : '');
    node.hidden = !message;
  }
  async function api(path, body) {
    const options = {cache: 'no-store', signal: AbortSignal.timeout(12000)};
    if (body !== undefined) Object.assign(options, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const response = await fetch(path, options);
    const value = await response.json();
    if (!response.ok) throw new Error(typeof value.error === 'string' ? value.error : value.message || `请求失败 (${response.status})`);
    return value;
  }
  function formatElapsed(seconds) {
    if (!finite(seconds)) return '—';
    seconds = Math.max(0, Math.floor(seconds));
    return seconds < 60 ? `${seconds} 秒` : `${Math.floor(seconds / 60)} 分 ${seconds % 60} 秒`;
  }
  function requestId() {
    if (crypto.randomUUID) return crypto.randomUUID();
    return Array.from(crypto.getRandomValues(new Uint8Array(16)), (x) => x.toString(16).padStart(2, '0')).join('');
  }
  function taskBusy() { return Boolean(state && (state.busy || state.task && busyStates.has(state.task.state))); }
  function taskCommand(task) {
    if (task.kind === 'validation') return '传感器检查';
    const command = task.command;
    let structured = command;
    if (typeof command === 'string') {
      try { structured = JSON.parse(command); } catch { return command || '机器人任务'; }
    }
    if (structured && typeof structured === 'object' && rooms[structured.room] && categories[structured.category] && ['find', 'count'].includes(structured.mode)) {
      return `去${rooms[structured.room]}${structured.mode === 'count' ? '统计' : '寻找'}${categories[structured.category]}`;
    }
    return typeof command === 'string' ? command : '机器人任务';
  }
  function refreshButtons() {
    const allowed = Boolean(online && state && state.ready && !taskBusy() && !mutationPending);
    el('task-fields').disabled = !allowed;
    el('submit-task').disabled = !allowed;
    text('submit-task', mutationPending ? '正在提交…' : taskBusy() ? '任务执行中' : allowed ? '开始任务' : '等待系统就绪');
    el('cancel-task').disabled = !online || mutationPending;
    el('validate').disabled = !allowed;
    text('task-help', !online ? '连接已断开，无法提交任务。' : taskBusy() ? '完成或停止当前任务后，可开始下一条。' : allowed ? '支持文字寻物与盘点，每次执行一个任务。' : '正在等待传感器、定位和搜索服务就绪。');
  }
  function healthRow(label, value, tone) {
    const li = document.createElement('li');
    const left = document.createElement('span'); left.textContent = label;
    const right = document.createElement('span'); right.textContent = value; right.className = 'status-label ' + tone;
    li.append(left, right);
    return li;
  }
  function renderHealth() {
    const telemetry = state.telemetry || {};
    const list = el('health'); list.replaceChildren();
    for (const [name, ready] of [['仿真数据', telemetry.simulation_ready], ['定位与导航', telemetry.navigation_ready], ['视觉数据', telemetry.perception_ready]]) {
      list.append(healthRow(name, ready ? '就绪' : '等待数据', ready ? 'good' : 'warn'));
    }
    for (const [name, component] of Object.entries(state.components || {})) {
      if (!Object.hasOwn(componentLabels, name)) continue;
      const current = component.state || (component.returncode != null ? 'exited' : component.pid ? 'running' : 'pending');
      list.append(healthRow(componentLabels[name] || name, current === 'running' ? '运行中' : statusLabel(current), current === 'running' ? 'good' : ['failed', 'exited'].includes(current) && component.returncode ? 'bad' : 'neutral'));
    }
    el('system-dot').className = 'dot ' + (state.error ? 'bad' : state.ready ? 'good' : 'warn');
    el('system-dot').setAttribute('aria-label', state.error ? '系统异常' : state.ready ? '系统就绪' : '等待就绪');
    const topics = el('topics'); topics.replaceChildren();
    for (const [name, info] of Object.entries(telemetry.topics || {})) {
      const row = document.createElement('li');
      const topic = document.createElement('code'); topic.textContent = name;
      const status = document.createElement('span');
      status.className = 'status-label ' + (info.fresh ? 'good' : 'warn');
      status.textContent = info.messages ? info.fresh ? `已收 ${info.messages} 条` : finite(info.age_seconds) ? `${info.age_seconds.toFixed(1)} 秒前` : '数据过期' : '尚无数据';
      row.append(topic, status); topics.append(row);
    }
    if (!topics.children.length) topics.append(healthRow('传感器', '等待数据', 'neutral'));
  }
  function appendArtifacts(container, artifacts) {
    container.replaceChildren();
    for (const artifact of Array.isArray(artifacts) ? artifacts : []) {
      if (!artifact || typeof artifact.url !== 'string') continue;
      let url;
      try { url = new URL(artifact.url, location.href); } catch { continue; }
      if (url.origin !== location.origin || url.pathname !== '/api/artifact') continue;
      const link = document.createElement('a');
      link.href = url.href;
      link.textContent = artifact.name || artifact.kind || '查看文件';
      link.target = '_blank'; link.rel = 'noopener';
      container.append(link);
    }
  }
  function selectedTask() {
    return selectedHistoryId ? (state.history || []).find((row) => row.id === selectedHistoryId) || state.task : state.task;
  }
  function renderTask() {
    const task = selectedTask();
    const historical = Boolean(selectedHistoryId && task && task.id === selectedHistoryId);
    text('task-title', historical ? '历史任务' : '当前任务');
    el('show-current').hidden = !historical;
    const result = task && task.result;
    const validation = task && task.kind === 'validation';
    const current = task ? task.state || 'running' : 'idle';
    badge('task-badge', current);
    text('active-command', task ? taskCommand(task) : '等待你的第一条指令');
    text('elapsed', task ? formatElapsed(task.elapsed_seconds) : '—');
    text('count', !validation && result && finite(result.count) ? String(result.count) : '—');
    const key = JSON.stringify([task && task.id, current, result, task && task.error, task && task.save_status, task && task.artifacts, historical]);
    if (taskKey === key) return;
    taskKey = key;
    let feedback = '选择房间和物品，系统就绪后即可开始。';
    if (task) {
      if (task.error) feedback = String(task.error);
      else if (current === 'canceling') feedback = '已请求停止，正在等待机器人确认。';
      else if (busyStates.has(current)) feedback = validation ? '正在检查实际传感器数据和坐标变换。' : '机器人正在执行指令，等待实际搜索结果。';
      else if (validation) feedback = result ? result.passed ? '传感器检查通过。详细数据已记录在报告中。' : '传感器检查未通过，请查看报告与日志。' : statusLabel(current);
      else if (result) {
        const resultLabel = statusLabel(result.status || current);
        feedback = `${resultLabel}${finite(result.count) ? `，已确认 ${result.count} 个${categories[result.category] || result.category || '目标'}。` : '。'}`;
      } else feedback = statusLabel(current);
      if (task.save_status === 'partial') feedback += '\n部分结果文件尚未完整保存，请查看日志。';
      if (task.save_status === 'missing') feedback += '\n未找到结果文件，请查看日志。';
      if (historical) feedback += '\n正在查看本次运行中的历史记录。';
    }
    text('task-feedback', feedback);
    const coverage = result && result.coverage;
    el('coverage').hidden = !coverage;
    if (coverage) {
      const observed = coverage.viewpoints_observed;
      const total = result.viewpoints_total;
      text('coverage', `${finite(observed) && finite(total) ? `有效观察位置 ${observed} / ${total}。` : ''}${coverage.full_room_coverage_verified === true ? '房间覆盖已验证。' : '数量来自已完成的有效观察，未确认全房覆盖。'}`);
    }
    const objects = el('objects'); objects.replaceChildren();
    for (const object of !validation && result && Array.isArray(result.objects) ? result.objects : []) {
      const row = document.createElement('div'); row.className = 'object-item';
      const id = object.id ?? object.object_id ?? '?';
      const xyz = ['x', 'y', 'z'].map((name) => finite(object[name]) ? object[name].toFixed(2) : '—').join(', ');
      row.textContent = `#${id} ${categories[object.category || result.category] || object.category || result.category || '物品'} · (${xyz}) m`;
      objects.append(row);
    }
    appendArtifacts(el('artifacts'), task && task.artifacts);
    el('result-details').hidden = !result;
    text('result-json', result ? JSON.stringify(result, null, 2) : '');
    drawMap();
  }
  function renderHistory() {
    const history = Array.isArray(state.history) ? state.history : [];
    text('history-count', `${history.length} 条`);
    const key = JSON.stringify(history.map((row) => [row.id, row.state, row.command, row.result && row.result.count]));
    if (key === historyKey) return;
    historyKey = key;
    const list = el('history'); list.replaceChildren();
    if (!history.length) {
      const row = document.createElement('li'); row.className = 'empty-row'; row.textContent = '完成的任务会显示在这里。'; list.append(row); return;
    }
    for (const task of history.slice(0, 20)) {
      const row = document.createElement('li'); const button = document.createElement('button');
      button.type = 'button'; button.className = 'history-item';
      const title = document.createElement('strong'); title.textContent = taskCommand(task);
      const meta = document.createElement('span'); meta.className = 'history-meta';
      meta.textContent = `${statusLabel(task.state)} · ${formatElapsed(task.elapsed_seconds)}${task.result && finite(task.result.count) ? ` · ${task.result.count} 个` : ''}`;
      button.append(title, meta);
      button.addEventListener('click', () => { selectedHistoryId = task.id; taskKey = ''; renderTask(); });
      row.append(button); list.append(row);
    }
  }
  function renderValidation() {
    const value = state.validation;
    if (!value) return;
    text('validation-state', busyStates.has(value.state) ? '正在检查传感器…' : value.error || (value.result && value.result.passed === true ? '最近一次传感器检查通过。' : value.result && value.result.passed === false ? '最近一次检查未通过，请查看报告。' : statusLabel(value.state)));
    appendArtifacts(el('validation-artifacts'), value.artifacts);
  }
  function renderRooms() {
    const definitions = state.rooms || {};
    const signature = JSON.stringify(definitions);
    if (signature === roomSignature) return;
    roomSignature = signature;
    const selected = el('room').value;
    el('room').replaceChildren();
    for (const key of Object.keys(rooms)) delete rooms[key];
    for (const [id, room] of Object.entries(definitions)) {
      rooms[id] = room.display_name || (room.aliases || [id])[0];
      const option = document.createElement('option'); option.value = id; option.textContent = rooms[id];
      el('room').append(option);
    }
    if (definitions[selected]) el('room').value = selected;
    taskKey = ''; historyKey = '';
  }
  function renderState() {
    renderRooms();
    badge('connection', 'ready', '已连接');
    text('phase', statusLabel(state.phase));
    text('session', `会话 ${state.session_id || '—'}`);
    if (state.error && state.error !== lastNotice) announce(String(state.error), true);
    renderHealth(); renderTask(); renderHistory(); renderValidation(); refreshButtons(); drawMap();
  }

  function setMap(value) {
    if (!value || !value.available) { mapUnavailable = true; drawMap(); return; }
    const {width, height, resolution, data} = value;
    if (!Number.isInteger(width) || !Number.isInteger(height) || width < 1 || height < 1 || width * height > 16000000 || !finite(resolution) || resolution <= 0 || !Array.isArray(data) || data.length !== width * height) return;
    const origin = value.origin || {};
    if (![origin.x, origin.y, origin.yaw].every(finite)) return;
    mapUnavailable = false;
    mapData = value;
    mapRaster = document.createElement('canvas'); mapRaster.width = width; mapRaster.height = height;
    const rasterContext = mapRaster.getContext('2d');
    const pixels = rasterContext.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const occupancy = data[y * width + x];
        const index = ((height - 1 - y) * width + x) * 4;
        const color = occupancy < 0 ? [219, 228, 231] : occupancy >= 65 ? [61, 82, 94] : occupancy === 0 ? [250, 252, 252] : [250 - Math.round(occupancy * 1.6), 252 - Math.round(occupancy * 1.5), 252 - Math.round(occupancy * 1.4)];
        pixels.data[index] = color[0]; pixels.data[index + 1] = color[1]; pixels.data[index + 2] = color[2]; pixels.data[index + 3] = 255;
      }
    }
    rasterContext.putImageData(pixels, 0, 0);
    el('map-empty').hidden = true;
    text('map-meta', `${value.frame_id || 'map'} · ${resolution.toFixed(2)} m / 栅格 · ${(width * resolution).toFixed(1)} × ${(height * resolution).toFixed(1)} m`);
    drawMap();
  }
  function drawMap() {
    const bounds = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    if (canvas.width !== Math.floor(bounds.width * ratio) || canvas.height !== Math.floor(bounds.height * ratio)) {
      canvas.width = Math.floor(bounds.width * ratio); canvas.height = Math.floor(bounds.height * ratio);
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, bounds.width, bounds.height);
    if (!mapData || !mapRaster) return;
    const scale = Math.min((bounds.width - 40) / mapData.width, (bounds.height - 40) / mapData.height) * zoom;
    const offsetX = (bounds.width - mapData.width * scale) / 2;
    const offsetY = (bounds.height - mapData.height * scale) / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapRaster, offsetX, offsetY, mapData.width * scale, mapData.height * scale);
    const origin = mapData.origin;
    const c = Math.cos(origin.yaw), s = Math.sin(origin.yaw);
    function project(x, y) {
      const dx = x - origin.x, dy = y - origin.y;
      return [offsetX + (c * dx + s * dy) / mapData.resolution * scale, offsetY + (mapData.height - (-s * dx + c * dy) / mapData.resolution) * scale];
    }
    if (state && state.rooms_frame_id === mapData.frame_id) {
      const definitions = state.rooms || {};
      if (el('show-regions').checked) for (const [id, room] of Object.entries(definitions)) {
        if (!Array.isArray(room.polygon) || room.polygon.length < 3) continue;
        const color = /^#[0-9a-f]{6}$/i.test(room.color || '') ? room.color : '#147d76';
        ctx.save(); ctx.beginPath();
        room.polygon.forEach((point, index) => {
          const p = project(point[0], point[1]);
          if (index === 0) ctx.moveTo(...p); else ctx.lineTo(...p);
        });
        ctx.closePath(); ctx.fillStyle = color; ctx.globalAlpha = .08; ctx.fill();
        ctx.globalAlpha = .9; ctx.strokeStyle = color; ctx.lineWidth = id === el('room').value ? 2.5 : 1.4; ctx.stroke();
        const label = room.label_position;
        if (Array.isArray(label) && label.every(finite)) {
          const p = project(...label); const name = rooms[id] || id;
          ctx.font = 'bold 13px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
          const width = ctx.measureText(name).width + 12;
          ctx.globalAlpha = .94; ctx.fillStyle = '#ffffff'; ctx.fillRect(p[0]-width/2,p[1]-11,width,22);
          ctx.globalAlpha = 1; ctx.fillStyle = color; ctx.fillText(name,...p);
        }
        ctx.restore();
      }
      const room = definitions[el('room').value];
      if (room && el('show-views').checked) {
        const views = (room.category_views || {})[el('category').value] || room.views || [];
        views.forEach((view, index) => {
          const [x,y] = project(view[0],view[1]); const heading = -(view[2]-origin.yaw);
          ctx.save(); ctx.strokeStyle = '#273e4b'; ctx.fillStyle = '#ffffff'; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(x,y,7,0,2*Math.PI); ctx.fill(); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(x,y); ctx.lineTo(x+17*Math.cos(heading),y+17*Math.sin(heading)); ctx.stroke();
          ctx.font='bold 11px sans-serif'; ctx.fillStyle='#273e4b'; ctx.fillText(String(index+1),x+9,y-9-index*2); ctx.restore();
        });
      }
    }
    const pose = state && state.telemetry && state.telemetry.pose;
    const poseFresh = online && pose && pose.fresh !== false && state.telemetry.pose_fresh !== false;
    if (pose && finite(pose.x) && finite(pose.y) && finite(pose.yaw) && (!pose.frame_id || pose.frame_id === mapData.frame_id)) {
      const [x, y] = project(pose.x, pose.y);
      const heading = -(pose.yaw - origin.yaw);
      ctx.save(); ctx.translate(x, y); ctx.rotate(heading);
      ctx.fillStyle = poseFresh ? '#147d76' : '#81969c'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(0, 0, 10, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.moveTo(7, 0); ctx.lineTo(-3, -4); ctx.lineTo(-3, 4); ctx.closePath(); ctx.fill(); ctx.restore();
      text('pose', `${poseFresh ? '' : '上次位置 · '}x ${pose.x.toFixed(2)} / y ${pose.y.toFixed(2)} m`);
    } else text('pose', '位置等待定位');
    const task = state && selectedTask();
    const result = task && task.kind !== 'validation' && task.result;
    const objects = result && Array.isArray(result.objects) ? result.objects : [];
    for (const object of objects) {
      if (!finite(object.x) || !finite(object.y)) continue;
      if (result.frame_id && result.frame_id !== mapData.frame_id) continue;
      const [x, y] = project(object.x, object.y);
      const id = String(object.id ?? object.object_id ?? '?');
      ctx.fillStyle = '#d38b23'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.font = '600 11px system-ui'; ctx.textAlign = 'center';
      const label = `#${id}`;
      const width = ctx.measureText(label).width + 12;
      ctx.fillStyle = '#fff'; ctx.fillRect(x - width / 2, y - 29, width, 18);
      ctx.fillStyle = '#8b601b'; ctx.fillText(label, x, y - 16);
    }
    text('map-result-context', result ? `${selectedHistoryId ? '历史' : '当前'}任务结果：${objects.length} 个已确认物品标记。机器人图标为最新可用定位。` : '物品标记将在任务结果返回后显示。');
    el('map-stale').hidden = online && !mapData.stale && !mapUnavailable;
    text('map-stale', !online ? '连接中断 · 显示最后收到的地图' : mapUnavailable ? '地图暂不可用 · 显示最后收到的地图' : '地图数据已过期');
  }

  async function pollState() {
    try { state = await api('/api/state'); online = true; renderState(); }
    catch (error) {
      online = false; badge('connection', 'error', '连接中断'); text('phase', '检查启动终端与服务状态');
      el('system-dot').className = 'dot bad'; refreshButtons(); drawMap();
    }
    setTimeout(pollState, 1000);
  }
  async function pollMap() {
    try { setMap(await api('/api/map')); } catch { mapUnavailable = true; drawMap(); }
    setTimeout(pollMap, 3000);
  }
  async function refreshLog() {
    if (logBusy) return;
    logBusy = true;
    try {
      const output = await api(`/api/log?component=${encodeURIComponent(el('log-component').value)}`);
      const target = el('log-output');
      const nearEnd = target.scrollHeight - target.scrollTop - target.clientHeight < 35;
      target.textContent = output.text || '该组件暂时没有日志。';
      if (nearEnd) target.scrollTop = target.scrollHeight;
    } catch (error) { text('log-output', `日志读取失败：${error.message}`); }
    finally { logBusy = false; }
  }
  async function mutate(path, body, message) {
    if (mutationPending) return;
    mutationPending = true; refreshButtons();
    try {
      await api(path, body);
      selectedHistoryId = null; taskKey = ''; announce(message);
      state = await api('/api/state'); online = true; renderState();
    } catch (error) { announce(error.message, true); }
    finally { mutationPending = false; refreshButtons(); }
  }
  el('task-form').addEventListener('submit', (event) => {
    event.preventDefault();
    if (!state || !state.ready || taskBusy() || mutationPending || !online) return;
    const command = el('command').value.trim();
    const body = command ? {command, request_id: requestId()} : {mode: el('mode').value, room: el('room').value, category: el('category').value, request_id: requestId()};
    mutate('/api/task', body, '任务已提交，执行状态将在右侧更新。');
  });
  el('cancel-task').addEventListener('click', () => mutate('/api/cancel', {}, '已提交停止请求，请等待执行状态更新。'));
  el('validate').addEventListener('click', () => mutate('/api/validate', {}, '开始传感器检查，报告将保存在本次会话中。'));
  el('show-current').addEventListener('click', () => { selectedHistoryId = null; taskKey = ''; renderTask(); });
  el('log-component').addEventListener('change', refreshLog);
  el('refresh-log').addEventListener('click', refreshLog);
  el('zoom-in').addEventListener('click', () => { zoom = Math.min(4, zoom * 1.2); drawMap(); });
  el('zoom-out').addEventListener('click', () => { zoom = Math.max(.5, zoom / 1.2); drawMap(); });
  el('fit-map').addEventListener('click', () => { zoom = 1; drawMap(); });
  ['room','category','show-regions','show-views'].forEach((id) => el(id).addEventListener('change', drawMap));
  new ResizeObserver(drawMap).observe(el('map-stage'));
  pollState(); pollMap(); refreshLog();
  setInterval(refreshLog, 5000);
})();
