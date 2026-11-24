#!/usr/bin/env python3
"""Package existing defense evidence without running or changing robot tasks."""
import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import shutil
import textwrap
import zipfile

from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = Path('results/development/demo_two_rooms_20261123')
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def read(path):
    return json.loads((ROOT / path).read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_cards():
    """Numbers and acceptance assertions come from preserved audit receipts."""
    cards = []
    for room, title in [('bedroom', '卧室：均匀清扫与回充'), ('utility', '杂物间：覆盖与局部补扫')]:
        base = EVIDENCE / 'cleaning_followup/repair' / room
        audit = read(base / 'audit.json')
        assert audit['single_case_passed'] and all(audit['case_gates'].values())
        c, s = audit['coverage'], audit['spatial_coverage']
        cards.append(dict(id=room, title=title, status='单次案例通过',
            facts=[f"实际覆盖 {100*c['covered_cells']/c['eligible_cells']:.2f}%",
                   f"实际刷到原条带 {s['covered_lanes']} / {s['planned_lanes']}",
                   f"已扫 {c['covered_cells']} / {c['eligible_cells']} 格",
                   f"任务耗时 {audit['elapsed_seconds']:.2f} 秒", '接触0 · 时效检查通过 · 回充'],
            limit='覆盖按实际刷盘计算；仍有未扫地面。单次所选配置，不代表全屋或全部配置稳定验收。',
            receipts=[base / 'audit.json'], assets=[base / 'presentation.png'],
            sessions=[Path(audit['session']).name]))

    base = EVIDENCE / 'route_pair'
    audit, window = read(base / 'audit.json'), read(base / 'task_window_audit.json')
    assert audit['baseline_evaluation']['passed'] and audit['blocked_evaluation']['passed']
    assert window['status'] == 'passed' and all(window['checks'].values())
    cards.append(dict(id='rerouting', title='途中受阻：自主改走北侧通路', status='代表任务窗通过',
        facts=['同起点、同厨房任务、同固定观察点', '无箱：实际走客厅较短路线',
               '途中放箱：感知后自主改走北侧', '厨房两处巡视 · 现场计数2/2 · 回充'],
        limit='北侧指靠近健身区的一侧，不要求进入健身区。首无箱任务失败、放箱准备期5次源超界均保留；不是全运行时稳定性通过。',
        receipts=[base / 'audit.json', base / 'task_window_audit.json', base / 'baseline/report.json'],
        assets=[base / 'actual_pair.png'], sessions=[audit['session_id']]))

    base = EVIDENCE / 'bowl_book'
    audit = read(base / 'audit.json')
    trials = audit['trials']['near_mixed']
    assert trials['all_checks_passed'] and len(trials['cases']) == 8
    counts = [c for c in trials['cases'] if c['mode'] == 'count']
    names = {'bowl': '厨房碗', 'book': '卧室书', 'cup': '杯'}
    facts = []
    for c in counts:
        e = c['evaluation']
        assert e['passed'] and all(e['checks'].values())
        name = names[c['category']]
        if c['category'] == 'cup': name = ('厨房' if c['room'] == 'kitchen' else '卧室') + name
        facts.append(f"{name}：{e['actual']}/{e['expected']}，最大误差{100*max(x['xy_error_m']for x in e['errors']):.1f}cm")
    cards.append(dict(id='counting', title='厨房与卧室：两处巡视、现场去重计数', status='所选布局首轮通过',
        facts=facts + ['寻找＋计数及原杯回归，共8项通过'],
        limit='厨房餐桌＋L形操作台；卧室书桌＋床头柜。S1280/书辅助/v14候选，额外布局与重复暂缓；不能称新增类稳定准入。',
        receipts=[base / 'audit.json'], assets=[
            base / 'frames_near/6b0501c9-f708-4d0d-abbe-277df069272d_view4/rgb.png',
            base / 'frames_near/31c5d62b-982b-4837-8d2e-e4e3b26f8423_view2/rgb.png'],
        sessions=[Path(trials['session']).name]))

    base = EVIDENCE / 'semantic_followup/movement_cup_diagnostic'
    audit = read(base / 'audit.json')
    assert audit['all_full_checks_passed'] and all(audit['memory_checks'].values())
    cards.append(dict(id='memory', title='物品变化：现场核验并更新语义记忆', status='单轮变化核验通过',
        facts=['历史记录提供线索，现场观测确认当前位置', '旧位置：历史／待核验；新位置：本次观测',
               '移动后寻找1个、完整计数2个并回充', 'SQLite与真实Spark-DSG重载一致'],
        limit='不声称移动物理身份重识别、旧位绝对为空或合格提速收益。首次定位失败保留；此例不是有/无记忆提效验收。',
        receipts=[base / 'audit.json'], assets=[base / 'memory_change.png',
          EVIDENCE / 'semantic_followup/recording_cup_diagnostic/ui_telemetry_4x.mp4'],
        sessions=[audit['session']]))

    base = EVIDENCE / 'map_exports/shared_views_verified'
    audit = read(base / 'audit.json')
    assert audit['one_logical_base'] is True and len(audit['records']) == 8
    for suffix, title, views in [
        ('space', '一份底图：栅格、分区、观察与代价', ['occupancy', 'rooms', 'observations', 'cost']),
        ('semantic', '语义与任务图层：物品、清扫与关系', ['semantic', 'clean_trace', 'clean_coverage', 'relations'])]:
        records = [next(r for r in audit['records'] if r['view'] == v) for v in views]
        cards.append(dict(id='maps_' + suffix, title=title, status='真实数据导出已核对',
            facts=['7类空间视图＋真实语义关系图', 'PNG／SVG／原始JSON，保留各自源戳',
                   '同一逻辑底图，任务编号与历史ID分开'],
            limit='这些视图来自标明的不同会话，不是同步快照。代价图为原生全局图；局部代价图未接入。',
            receipts=[base / 'audit.json'], assets=[Path(p) for r in records for p in r['paths']],
            sessions=sorted({r['session'] for r in records})))

    base = EVIDENCE / 'cleaning_followup/ball_trial/repair_followup'
    audit = read(base / 'audit.json')
    assert audit['full_cleaning_passed'] is False and audit['encounter_subchecks_passed'] is True
    c, s = audit['coverage'], audit['spatial_coverage']
    cards.append(dict(id='balls_partial', title='附录：散落球避障与清扫边界', status='部分完成，完整清扫未通过',
        facts=[f"实际覆盖 {100*c['covered_cells']/c['eligible_cells']:.2f}%（目标80%）",
               f"实际刷到原条带 {s['covered_lanes']} / {s['planned_lanes']}",
               '障碍检出、绕行、继续清扫、回充子项通过', '剩余可达区域仍有漏扫'],
        limit='保留partial，不把未扫部分全部归为球占据。片段从任务中途开始，是4倍UI遥测，不是相机录像或完整任务。',
        receipts=[base / 'audit.json'], assets=[base / 'remaining_snapshot.png', base / 'recording/ui_telemetry_4x.mp4'],
        sessions=[audit['session']]))
    return cards


def draw_slide(card, output):
    canvas = Image.new('RGB', (1920, 1080), '#f4f7fa')
    draw = ImageDraw.Draw(canvas)
    def write(x, y, text, size=28, fill='#244253'):
        draw.text((x, y), text, font=ImageFont.truetype(FONT, size), fill=fill)
    def wrapped(text, width, size):
        font = ImageFont.truetype(FONT, size)
        line = ''
        for character in text:
            if line and draw.textlength(line + character, font=font) > width:
                yield line
                line = ''
            line += character
        if line: yield line
    draw.rectangle((0, 0, 1920, 12), fill='#087f80')
    write(62, 35, card['title'], 46)
    write(65, 102, card['status'], 27, '#9a5e00' if '部分' in card['status'] else '#087f80')
    pictures = [p for p in card['assets'] if p.suffix.lower() == '.png']
    if card['id'].startswith('maps_'):
        for i, path in enumerate(pictures):
            image = ImageOps.contain(Image.open(ROOT / path).convert('RGB'), (860, 350))
            canvas.paste(image, (65 + (i % 2)*925 + (860-image.width)//2, 158+(i//2)*367))
    elif card['id'] == 'counting':
        for i, (path, label) in enumerate(zip(pictures, ['厨房计数：餐桌观察实帧', '卧室计数：书桌观察实帧'])):
            image = ImageOps.contain(Image.open(ROOT / path).convert('RGB'), (570, 470))
            x = 655 + i*600
            canvas.paste(image, (x+(570-image.width)//2, 275+(470-image.height)//2))
            write(x, 780, label, 26)
    else:
        image = ImageOps.contain(Image.open(ROOT / pictures[0]).convert('RGB'), (1190, 740))
        canvas.paste(image, (655+(1190-image.width)//2, 165+(740-image.height)//2))
    if not card['id'].startswith('maps_'):
        y = 215
        for fact in card['facts']:
            for line in wrapped(fact, 540, 29):
                write(65, y, line, 29)
                y += 47
            y += 24
    y = 923
    for line in wrapped(card['limit'], 1780, 25):
        write(65, y, line, 25, '#65543f')
        y += 35
    write(65, 1040, '原图、审计与冻结配置见离线目录；不同会话不拼称连续验收。', 20, '#627584')
    canvas.save(output)


def package(output):
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema=1, created_utc=datetime.now(timezone.utc).isoformat(),
                    scope='Selected evidence delivery; not new task or full-system acceptance.', cards=[], files=[])
    def copy(source, target):
        path = (ROOT / source).resolve()
        path.relative_to(ROOT)
        target = output / target
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            assert sha(target) == sha(path)
        else:
            shutil.copyfile(path, target)
            manifest['files'].append(dict(original=str(path.relative_to(ROOT)),
                path=str(target.relative_to(output)), bytes=target.stat().st_size, sha256=sha(target)))
        return str(target.relative_to(output))
    cards = selected_cards()
    for index, card in enumerate(cards, 1):
        row = {k: v for k, v in card.items() if k not in ('assets', 'receipts')}
        row['assets'] = [copy(p, Path('assets') / card['id'] / p.relative_to(EVIDENCE)) for p in card['assets']]
        row['receipts'] = [copy(p, Path('evidence') / p.relative_to(EVIDENCE)) for p in card['receipts']]
        row['slide'] = f"slides/{index:02d}_{card['id']}.png"
        (output / 'slides').mkdir(exist_ok=True)
        draw_slide(card, output / row['slide'])
        manifest['cards'].append(row)
    sessions = {}
    for session in sorted({s for c in cards for s in c['sessions']}):
        base = Path('results/sessions') / session
        meta = read(base / 'session.json')
        copy(base / 'session.json', Path('sessions') / session / 'session.json')
        for group in ('configuration_snapshots', 'source_snapshots'):
            for original, snapshot in meta[group].items():
                assert sha(ROOT / base / snapshot) == meta['source_sha256'][original], original
                copy(base / snapshot, Path('sessions') / session / snapshot)
        sessions[session] = dict(configuration=meta['config'], frozen_hashes_checked=True,
                                original_runtime_note=meta.get('note'))
    manifest['sessions'] = sessions
    copy(Path(__file__).relative_to(ROOT), Path('provenance/package_tool.py'))
    copy(Path('docs/maintained/SOURCES.md'), Path('provenance/SOURCES.md'))
    page = ['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>家庭机器人 · 答辩素材</title>',
      '<style>body{margin:0;background:#eff4f7;color:#1f3545;font:17px/1.6 sans-serif}main{max-width:1400px;margin:auto;padding:30px}h1{margin:0}header,article{background:white;padding:28px;margin-bottom:24px;border-radius:16px}a{color:#007476}nav{display:flex;flex-wrap:wrap;gap:16px;margin:20px 0}.slide{width:100%;height:auto;border:1px solid #d8e3e9}video{width:min(100%,800px)}.scope{padding:12px;background:#fff5de}.sources{font-size:14px;overflow-wrap:anywhere}@media print{header,nav,video,.sources{display:none}article{break-after:page;padding:0;margin:0}.slide{width:100%}}</style><main>',
      '<header><h1>家庭服务机器人 · 答辩素材</h1><p>已有真实执行结果的选材目录。每项保留原会话、配置、结果范围和失败限制；不同轮次分开展示。</p><p>下载PNG页可插入PPT，原图可放大，原始JSON可核对。此包不代表新增整机验收。</p><a href="manifest.json">文件散列与冻结配置清单</a></header><nav>']
    escape = html.escape
    for card in manifest['cards']:
        page.append(f'<a href="#{card["id"]}">{escape(card["title"])}</a>')
    page.append('</nav>')
    for card in manifest['cards']:
        page.append(f'<article id="{card["id"]}"><h2>{escape(card["title"])}</h2><p>{escape(card["status"])}</p><img class="slide" src="{card["slide"]}" alt="{escape(card["title"])}"><p><a download href="{card["slide"]}">下载1920×1080素材页</a></p><p class="scope">{escape(card["limit"])}</p><ul>')
        for fact in card['facts']: page.append(f'<li>{escape(fact)}</li>')
        page.append('</ul><div class="sources">')
        for path in card['assets'] + card['receipts']:
            page.append(f'<p><a href="{escape(path)}">{escape(path)}</a></p>')
            if path.endswith('.mp4'): page.append(f'<video controls preload="none" src="{escape(path)}"></video>')
        page.append('<p>原会话：' + escape(' / '.join(card['sessions'])) + '</p></div></article>')
    page.append('</main></html>')
    (output / 'index.html').write_text('\n'.join(page))
    for path in sorted((output / 'slides').glob('*.png')):
        manifest['files'].append(dict(original=None, path=str(path.relative_to(output)), bytes=path.stat().st_size, sha256=sha(path)))
    index = output / 'index.html'
    manifest['files'].append(dict(original=None, path='index.html', bytes=index.stat().st_size, sha256=sha(index)))
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    # Verify every copied artifact and all local page links before delivery.
    from html.parser import HTMLParser
    class Links(HTMLParser):
        def handle_starttag(self, tag, attrs):
            for key, value in attrs:
                if key in ('src', 'href') and value and not value.startswith('#'):
                    resolved = (output / value).resolve()
                    resolved.relative_to(output.resolve())
                    assert resolved.is_file(), value
    Links().feed(index.read_text())
    for row in manifest['files']:
        assert sha(output / row['path']) == row['sha256']
    archive = output.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as zipped:
        for path in sorted(output.rglob('*')):
            if path.is_file(): zipped.write(path, str(Path(output.name) / path.relative_to(output)))
    with zipfile.ZipFile(archive) as zipped:
        assert zipped.testzip() is None
    print(json.dumps(dict(output=str(output), cards=len(cards), files=len(manifest['files']),
        sessions=len(sessions), archive=str(archive), archive_sha256=sha(archive)), ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    package(args.output)
