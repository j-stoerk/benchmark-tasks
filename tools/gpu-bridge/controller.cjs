// Manual owner-only GPU execution. No PR events, inbound ports, or stored PATs.
const fs = require('node:fs');
const path = require('node:path');
const {spawn, execFileSync} = require('node:child_process');
const {DefaultArtifactClient} = require('/tmp/robocurve-artifact/node_modules/@actions/artifact');
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const root = '/data/robocurve-evidence';
const workspace = path.resolve('.');
const artifact = new DefaultArtifactClient();
let totalBytes = 0;

async function upload(name, files, base) {
  files = files.filter(f => fs.existsSync(f) && fs.statSync(f).isFile());
  if (!files.length) return;
  const bytes = files.reduce((sum, f) => sum + fs.statSync(f).size, 0);
  if (totalBytes + bytes > 200e6) throw Error('200 MB evidence transport cap reached');
  const result = await artifact.uploadArtifact(name, files, base, {retentionDays: 1, compressionLevel: 6});
  totalBytes += bytes;
  console.log(JSON.stringify({artifact: name, bytes, id: result.id}));
}

function walk(dir) {
  if (!fs.existsSync(dir)) return [];
  return fs.readdirSync(dir, {withFileTypes: true}).flatMap(e => {
    const p = path.join(dir, e.name);
    return e.isDirectory() ? walk(p) : [p];
  });
}

module.exports = async ({github, context, core}) => {
  fs.mkdirSync(root, {recursive: true});
  const owner = context.repo.owner, repo = context.repo.repo;
  if (context.actor !== owner) throw Error('Only repository owner may operate this worker');
  async function control() {
    const response = await github.rest.repos.getContent({owner, repo, path: '.robocurve/control.json', ref: 'main'});
    return JSON.parse(Buffer.from(response.data.content, 'base64').toString('utf8'));
  }
  const completed = new Set();
  const deadline = Date.now() + 165 * 60e3;
  console.log('HTTPS evidence worker ready; hard stop after 165 minutes.');
  while (Date.now() < deadline) {
    const state = await control();
    const command = state.command;
    if (!command || completed.has(command.id)) {await sleep(6000); continue;}
    if (!/^[a-z0-9-]{1,64}$/.test(command.id)) throw Error('Invalid command ID');
    if (command.mode === 'exit') break;
    if (command.mode === 'cancel') {
      completed.add(command.id);
      core.info(`Cancellation ${command.id}: no matching running trial.`);
      continue;
    }
    if (!['probe', 'run'].includes(command.mode)) throw Error('Unsupported command');
    completed.add(command.id);
    const out = path.join(root, command.id);
    fs.mkdirSync(out, {recursive: false});
    fs.writeFileSync(path.join(out, 'command.json'), JSON.stringify(command, null, 2));
    const bridge = path.join(out, 'bridge');
    fs.mkdirSync(bridge);
    let args;
    if (command.mode === 'probe') {
      args = [path.join(workspace, 'tools/gpu-bridge/probe.sh')];
    } else {
      if (!/^[0-9a-f]{40}$/.test(command.sha || '')) throw Error('An immutable task SHA is required');
      const suite = ['industrial-roll-handoff', 'calender-loaded-nip-parallelism',
        'powder-transfer-split-valve', 'slurry-filter-isolate-service', 'slot-die-shim-manifold-setup',
        'collector-foil-roller-threading', 'slitter-blade-spacer-recipe', 'collector-foil-butt-splice',
        'web-tension-idler-calibration', 'secondary-dryer-door-vacuum-test',
        'lab-film-drawdown', 'lab-stirbar-recovery', 'lab-wet-mount-coverslip',
        'lab-syringe-membrane-filtration', 'lab-liquid-phase-separation',
        'safety-swollen-cell-charge', 'lab-peristaltic-tube-loading'];
      if (!suite.includes(command.task)) throw Error('Task not allowlisted');
      const trial = command.trial || {};
      if (!['smoke', 'demo', 'model'].includes(trial.mode)) throw Error('Invalid trial mode');
      if (!Number.isInteger(trial.max_steps) || trial.max_steps < 1 || trial.max_steps > 2400) throw Error('Invalid step cap');
      if (command.task === 'industrial-roll-handoff' && ![1000, 6000].includes(trial.lift_force)) throw Error('Invalid lift setting');
      if (trial.no_expand !== undefined && typeof trial.no_expand !== 'boolean') throw Error('Invalid grip setting');
      if (trial.proof_scale !== undefined && ![0, 1].includes(trial.proof_scale)) throw Error('Invalid proof scale');
      for (const key of ['unloaded_only', 'calibration_bench']) {
        if (trial[key] !== undefined && typeof trial[key] !== 'boolean') throw Error(`Invalid ${key}`);
      }
      if (trial.variant !== undefined && !/^[a-z0-9-]{1,40}$/.test(trial.variant)) throw Error('Invalid control variant');
      if (trial.mode === 'model' && (trial.no_expand || trial.unloaded_only || trial.calibration_bench ||
          (trial.variant && trial.variant !== 'normal'))) throw Error('Model cannot use scripted controls');
      const source = path.join(out, 'source');
      const gitEnv = {...process.env, GIT_LFS_SKIP_SMUDGE: '1', GIT_TERMINAL_PROMPT: '0'};
      execFileSync('git', ['clone', '--filter=blob:none', '--no-checkout', `https://github.com/${owner}/${repo}.git`, source], {env: gitEnv, stdio: 'inherit'});
      execFileSync('git', ['sparse-checkout', 'set', `tasks/${command.task}`], {cwd: source, env: gitEnv, stdio: 'inherit'});
      execFileSync('git', ['checkout', command.sha], {cwd: source, env: gitEnv, stdio: 'inherit'});
      args = [path.join(workspace, 'tools/gpu-bridge/trial.sh'), path.join(source, 'tasks', command.task), trial.mode,
        path.join(out, 'trial'), bridge, String(trial.max_steps), JSON.stringify(trial)];
    }
    const consolePath = path.join(out, 'worker.txt');
    const log = fs.openSync(consolePath, 'w');
    const child = spawn('bash', args, {stdio: ['ignore', log, log], detached: true});
    let exitCode = null;
    child.on('exit', (code, signal) => {exitCode = code === null ? `signal:${signal}` : code;});
    child.on('error', error => {exitCode = `spawn-error:${error.code}`;});
    await upload(`${command.id}-started`, [path.join(out, 'command.json')], out);
    let cancellationReason = null;
    const uploaded = new Set();
    let nextProgress = Date.now() + 90e3, progressIndex = 0;
    while (exitCode === null) {
      const current = await control();
      if (Date.now() >= deadline || current.command?.mode === 'exit' ||
          (current.command?.mode === 'cancel' && current.command?.target === command.id)) {
        cancellationReason = Date.now() >= deadline ? 'worker time limit' :
          current.command.mode === 'exit' ? 'worker exit requested' : 'named diagnostic cancelled';
        if (current.command?.mode === 'cancel') completed.add(current.command.id);
        // Only this worker's own detached trial process group may be stopped.
        try {process.kill(-child.pid, 'SIGTERM');} catch (error) {if (error.code !== 'ESRCH') throw error;}
        const stopDeadline = Date.now() + 15000;
        while (exitCode === null && Date.now() < stopDeadline) await sleep(250);
        if (exitCode === null) {
          try {process.kill(-child.pid, 'SIGKILL');} catch (error) {if (error.code !== 'ESRCH') throw error;}
          await sleep(1000);
        }
        break;
      }
      for (const f of walk(bridge).filter(p => /request-\d+\.json$/.test(p))) {
        const base = path.basename(f, '.json');
        if (!uploaded.has(base)) {
          await upload(`${command.id}-${base}`, [f, ...walk(bridge).filter(p => path.basename(p).startsWith(base + '-') && p.endsWith('.png'))], bridge);
          uploaded.add(base);
        }
        const number = base.slice('request-'.length);
        const response = current.responses?.[`${command.id}:${number}`];
        const destination = path.join(bridge, `response-${number}.json`);
        if (response && !fs.existsSync(destination)) {
          fs.writeFileSync(destination + '.tmp', JSON.stringify(response));
          fs.renameSync(destination + '.tmp', destination);
        }
      }
      if (Date.now() >= nextProgress) {
        const tail = path.join(out, 'progress.txt');
        const candidates = [consolePath, ...walk(path.join(out, 'trial')).filter(p => p.endsWith('console.txt'))];
        fs.writeFileSync(tail, candidates.map(p => `${path.basename(p)}\n${fs.readFileSync(p, 'utf8').slice(-24000)}`).join('\n'));
        await upload(`${command.id}-progress-${++progressIndex}`, [tail], out);
        nextProgress = Date.now() + 120e3;
      }
      await sleep(5000);
    }
    fs.closeSync(log);
    fs.writeFileSync(path.join(out, 'worker-result.json'), JSON.stringify({exit_code: exitCode,
      cancelled: cancellationReason !== null, cancellation_reason: cancellationReason,
      command, finished_utc: new Date().toISOString()}, null, 2));
    const evidence = walk(out).filter(p => !p.includes('/source/') && /\.(json|jsonl|txt|png|mp4)$/.test(p));
    await upload(`${command.id}-result`, evidence, out);
    console.log(`Completed ${command.id}: ${exitCode}`);
  }
  core.info('GPU evidence controller stopped.');
};
